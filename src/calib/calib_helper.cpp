/*
 * OA-LICalib:
 * Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems
 *
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Xingxing Zuo
 * Copyright (C) 2022 Kewei Hu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <calib/calib_helper.h>

namespace liso {

LICalibrHelper::LICalibrHelper(const YAML::Node& node)
    : calib_step_(Start), iteration_num_(0) {
  double knot_distance = node["knot_distance"].as<double>();

  // data association parameters
  double associated_radius = 0.05;
  double plane_lambda = 0.6;

  // [1] Dataset
  LoadDataset(node);

  // [2] Optimization result
  calib_param_manager_ = std::make_shared<CalibParamManager>(node);

  p_LinI_backup_.resize(segment_dataset_->SegmentNum());
  so3_LtoI_backup_.resize(segment_dataset_->SegmentNum());
  for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
    p_LinI_backup_.at(id) = calib_param_manager_->p_LinI;
    so3_LtoI_backup_.at(id) = calib_param_manager_->so3_LtoI;
  }

  // [3] Class for each segment
  for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
    double traj_end = segment_dataset_->GetEndTime(id) -
                      segment_dataset_->GetStartTime(id) + 1e-9;

    std::shared_ptr<Trajectory> trajectory =
        std::make_shared<Trajectory>(knot_distance, 0, id);
    trajectory->extendKnotsTo(traj_end, SO3d(Eigen::Quaterniond::Identity()),
                              Eigen::Vector3d(0, 0, 0));
    trajectory->SetCalibParam(calib_param_manager_);
    trajectory_vec_.emplace_back(trajectory);

    SurfelAssociation::Ptr surfel_association =
        std::make_shared<SurfelAssociation>(associated_radius, plane_lambda);
    surfel_association_vec_.emplace_back(surfel_association);

    std::shared_ptr<ScanUndistortion> scan_undistortion =
        std::make_shared<ScanUndistortion>();
    scan_undistortion_vec_.emplace_back(scan_undistortion);

    if (!calib_param_manager_->locator_segment_param.empty()) {
      LIDARLocalization::Ptr ndt_locator = std::make_shared<LIDARLocalization>(
          calib_param_manager_->lo_param.ndt_resolution,
          calib_param_manager_->lo_param.ndt_key_frame_downsample,
          calib_param_manager_->lo_param.map_downsample_size,
          calib_param_manager_->locator_segment_param[id].locator_init_pose);
      ndt_locator->SetPriorMap(
          calib_param_manager_->locator_segment_param[id].ndt_prior_map_path);
      locator_vec_.emplace_back(ndt_locator);
    }
  }

  if (!CreateCacheFolder(segment_dataset_->GetSegmentBagPath().at(0))) {
    calib_step_ = Error;
  }
}

void LICalibrHelper::LoadDataset(const YAML::Node& node) {
  std::string lidar_model = node["LidarModel"].as<std::string>();
  LidarModelType lidar_model_type;

  if (lidar_model == "VLP_16_packet") {
    lidar_model_type = LidarModelType::VLP_16_packet;
  } else if (lidar_model == "VLP_16_SIMU") {
    lidar_model_type = LidarModelType::VLP_16_SIMU;
  } else if (lidar_model == "VLP_16_points") {
    lidar_model_type = LidarModelType::VLP_16_points;
  } else if (lidar_model == "VLP_32E_points") {
    lidar_model_type = LidarModelType::VLP_32E_points;
  } else if (lidar_model == "Ouster_16_points") {
    lidar_model_type = LidarModelType::Ouster_16_points;
  } else if (lidar_model == "Ouster_32_points") {
    lidar_model_type = LidarModelType::Ouster_32_points;
  } else if (lidar_model == "Ouster_64_points") {
    lidar_model_type = LidarModelType::Ouster_64_points;
  } else if (lidar_model == "Ouster_128_points") {
    lidar_model_type = LidarModelType::Ouster_128_points;
  } else if (lidar_model == "RS_16") {
    lidar_model_type = LidarModelType::RS_16;
  } else {
    calib_step_ = Error;
    ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
  }

  segment_dataset_ =
      std::make_shared<SegmentDatasetManager>(node, lidar_model_type);

  topic_imu_ = node["topic_imu"].as<std::string>();
  topic_lidar_ = node["topic_lidar"].as<std::string>();
}

bool LICalibrHelper::CreateCacheFolder(const std::string& bag_path) {
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    return false;
  }
  cache_path_parent_ = p.parent_path().string();
  cache_path_ = p.parent_path().string() + "/" + p.stem().string();
  boost::filesystem::create_directory(cache_path_);
  bag_name_ = p.stem().string();
  return true;
}

bool LICalibrHelper::CheckCalibStep(CalibStep desired_step,
                                    std::string func_name) const {
  static std::string step_descri[] = {"Error",
                                      "Start",
                                      "InitializationDone"
                                      "DataAssociationInOdomDone",
                                      "BatchOptimizationDone",
                                      "DataAssociationInRefinementDone",
                                      "RefineDone"};
  static TicToc timer;
  static std::string last_func_name = "Start";

  if (!ros::ok()) return false;

  bool check_pass = true;
  if (calib_step_ != desired_step) {
    check_pass = false;
    ROS_WARN("[%s] Need status: [%s].", func_name.c_str(),
             step_descri[int(desired_step)].c_str());
  }

  if (calib_step_ != Start && check_pass) {
    std::cout << GREEN << "\t[" << last_func_name << "] costs " << timer.toc()
              << " ms\n\t[" << func_name << "] starts ...\n"
              << RESET;
    last_func_name = func_name;
    timer.tic();
  }

  return check_pass;
}

void LICalibrHelper::Initialization() {
  if (!CheckCalibStep(Start, "Initialization")) return;

  for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
    CalibTool::InitialSO3TrajWithGyro(calib_param_manager_,
                                      segment_dataset_->GetImuData(id),
                                      trajectory_vec_.at(id));
  }

  // ros::Rate rate(0.5);
  // for (auto& ndt_locator : locator_vec_) {
  //   while (!ndt_locator->HasInitialPose()) {
  //     ndt_locator->PublishCloudAndOdom(
  //         segment_dataset_->GetScanData(0).front().full_features);
  //     rate.sleep();
  //   }
  // }

  /// use first segment to initialze extrinsic rotation
  if (!calib_param_manager_->calib_option.is_plane_motion) {
    bool ret = CalibTool::EstimateRotation(
        segment_dataset_->GetImuData(0), segment_dataset_->GetScanData(0),
        trajectory_vec_.at(0), calib_param_manager_);
    if (ret)
      calib_step_ = InitializationDone;
    else
      ROS_WARN("[Initialization] fails.");
  } else {
    ROS_WARN("[Initialization] skip.");
  }

  /// if initiailization fails, then use the prior
  if (calib_step_ != InitializationDone) {
    calib_step_ = InitializationDone;

    Eigen::Quaterniond q_ItoL = calib_param_manager_->q_LtoI.inverse();
    Eigen::Vector3d euler_ItoL = q_ItoL.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d euler_LtoI =
        calib_param_manager_->q_LtoI.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "[Initialization] rotation extrinsic set as default "
                 "parameters (degree); \neuler_ItoL: "
              << (euler_ItoL * 180.0 / M_PI).transpose()
              << "; euler_LtoI: " << (euler_LtoI * 180.0 / M_PI).transpose()
              << "\n";
  }
}

void LICalibrHelper::DataAssociationInOdom() {
  if (!CheckCalibStep(InitializationDone, "DataAssociationInOdom")) return;

  bool full_ret = true;
  for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
    bool seg_ret = CalibTool::DataAssociationWithOdom(
        segment_dataset_->GetScanData(id),
        segment_dataset_->GetScanTimestamps(id),
        segment_dataset_->GetSegmentTimestamp().at(id), trajectory_vec_.at(id),
        calib_param_manager_, scan_undistortion_vec_.at(id),
        surfel_association_vec_.at(id), cache_path_);

    full_ret = full_ret && seg_ret;

    trajectory_vec_.at(id)->SaveTrajectoryControlPoints(
        cache_path_ + "/odom_init_spline.txt");

    auto v = &segment_dataset_->GetSegmentTimestamp().at(id);
    double relative_start_time = v->first;
    double relative_end_time = v->second;

    trajectory_vec_.at(id)->TrajectoryToTUMTxt2(
        cache_path_ + "/odom_init_trajectory.txt", relative_start_time,
        relative_end_time);
  }

  if (full_ret) calib_step_ = DataAssociationInOdomDone;
}

void LICalibrHelper::DataAssociationInLocator() {
  if (!CheckCalibStep(InitializationDone, "DataAssociationInLocator")) return;

  bool full_ret = true;
  for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
    bool seg_ret = CalibTool::DataAssociationWithLocator(
        segment_dataset_->GetScanData(id),
        segment_dataset_->GetScanTimestamps(id),
        segment_dataset_->GetSegmentTimestamp().at(id), trajectory_vec_.at(id),
        calib_param_manager_,  // input
        locator_vec_.at(id), scan_undistortion_vec_.at(id),
        surfel_association_vec_.at(id), cache_path_);

    full_ret = full_ret && seg_ret;

    trajectory_vec_.at(id)->SaveTrajectoryControlPoints(
        cache_path_ + "/locator_init_spline.txt");

    auto v = &segment_dataset_->GetSegmentTimestamp().at(id);
    double relative_start_time = v->first;
    double relative_end_time = v->second;

    trajectory_vec_.at(id)->TrajectoryToTUMTxt2(
        cache_path_ + "/locator_init_trajectory.txt", relative_start_time,
        relative_end_time);
  }

  if (full_ret) calib_step_ = DataAssociationInOdomDone;
}

void LICalibrHelper::DataAssociationInRefinement() {
  bool ret = true;
  if (iteration_num_ == 1) {
    ret = CheckCalibStep(BatchOptimizationDone, "DataAssociationInRefinement");
  } else {
    ret = CheckCalibStep(RefineDone, "DataAssociationInRefinement");
  }
  if (!ret) return;

  bool full_ret = true;
  for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
    SetSegmentExtrinsic(id);

    bool seg_ret = CalibTool::DataAssociationWithTraj(
        segment_dataset_->GetScanData(id), trajectory_vec_.at(id),
        calib_param_manager_, scan_undistortion_vec_.at(id),
        surfel_association_vec_.at(id));
    full_ret = full_ret && seg_ret;
  }

  if (full_ret) calib_step_ = DataAssociationInRefinementDone;
}

void LICalibrHelper::BatchOptimization() {
  if (!CheckCalibStep(DataAssociationInOdomDone, "BatchOptimization")) return;

  std::cout << "\n====== Iteration " << iteration_num_ << " ======\n";

  TrajectoryEstimatorOptions options;
  options.lock_P = false;
  options.lock_R = false;
  options.lock_t_offset = false;

  options.lock_wb = false;
  options.lock_ab =
      calib_param_manager_->calib_option.lock_opt_first_accel_bias;
  options.lock_g = false;

  TrajInitFromSurfel(options);

  calib_step_ = BatchOptimizationDone;
  SaveCalibResult(cache_path_parent_ + "/calib_result.csv");
}

void LICalibrHelper::Refinement() {
  iteration_num_++;
  std::cout << "\n====== Iteration " << iteration_num_ << " ======\n";

  DataAssociationInRefinement();

  if (!CheckCalibStep(DataAssociationInRefinementDone, "Refinement")) return;

  TrajectoryEstimatorOptions options;
  options.lock_P = false;
  options.lock_R = false;
  options.lock_t_offset = !calib_param_manager_->calib_option.opt_time_offset;
  options.t_offset_padding =
      calib_param_manager_->calib_option.time_offset_padding;

  options.lock_wb = false;
  options.lock_ab = false;
  options.lock_g = false;

  // use gui to control intrinsic calib
  options.lock_IMU_intrinsic =
      !calib_param_manager_->calib_option.opt_IMU_intrinsic;
  options.lock_LiDAR_intrinsic =
      !calib_param_manager_->calib_option.opt_lidar_intrinsic;

  //  calib_param_manager_->ResetTimeOffset();
  TrajInitFromSurfel(options);

  calib_step_ = RefineDone;
  SaveCalibResult(cache_path_parent_ + "/calib_result.csv");

  trajectory_vec_.at(0)->SaveTrajectoryControlPoints(
      cache_path_ + "/trajectory_control_points.txt");
}

void LICalibrHelper::TrajInitFromSurfel(
    const TrajectoryEstimatorOptions& options) {
  // prepare associated lidar point
  int segment_num = segment_dataset_->SegmentNum();
  std::vector<Eigen::aligned_vector<PointCorrespondence>> point_measurement_vec;
  std::vector<std::pair<double, double>> valid_time_vec;
  point_measurement_vec.resize(segment_num);
  valid_time_vec.resize(segment_num);

  for (size_t id = 0; id < segment_num; ++id) {
    std::pair<double, double> selected_time(trajectory_vec_.at(id)->minTime(),
                                            trajectory_vec_.at(id)->maxTime());
    CalibTool::GetLidarPointCorrespondence(
        surfel_association_vec_.at(id), selected_time,
        point_measurement_vec.at(id), valid_time_vec.at(id));

    // if (segment_num > 1)
    {
      SetSegmentExtrinsic(id);

      std::shared_ptr<TrajectoryEstimator> estimator;
      estimator = std::make_shared<TrajectoryEstimator>(
          trajectory_vec_.at(id), calib_param_manager_, options);

      CalibTool::AddIMUAndSurfelToProblem(
          calib_param_manager_->calib_weights, segment_dataset_->GetImuData(id),
          point_measurement_vec.at(id), valid_time_vec.at(id),
          trajectory_vec_.at(id), estimator);

      ceres::Solver::Summary summary = estimator->Solve(50, false);

      std::cout << summary.BriefReport() << std::endl;

      ExtrinsicBackup(id);
    }
  }

  /// ********** output ********** ///
  std::cout << "Iteration, " << iteration_num_ << std::endl;
  std::cout << bag_name_ << std::endl;

  calib_param_manager_->showStates();

  std::cout << YELLOW;
  Eigen::Vector3d plane_cov =
      CalibTool::GetLidarCov(point_measurement_vec.at(0));
  CalibParamManager::friendly_output(plane_cov, 3, "plane cov", 3);
  std::cout << RESET;

  /// save data
  for (size_t id = 0; id < segment_num; ++id) {
    auto v = &segment_dataset_->GetSegmentTimestamp().at(id);
    double relative_start_time = v->first;
    double relative_end_time = v->second;

    trajectory_vec_.at(id)->TrajectoryToTUMTxt(
        cache_path_, relative_start_time, relative_end_time, iteration_num_);
  }

  TrajectoryViewer::PublishLoamCorrespondence(trajectory_vec_.at(0),
                                              point_measurement_vec.at(0));
  TrajectoryViewer::PublishIMUData(trajectory_vec_.at(0),
                                   segment_dataset_->GetImuData(0));
  TrajectoryViewer::PublishSplineTrajectory(
      trajectory_vec_.at(0), trajectory_vec_.at(0)->minTime(),
      trajectory_vec_.at(0)->maxTime(), 0.02);
  SavePointCloud();
}

void LICalibrHelper::SaveCalibResult(
    const std::string& calib_result_file) const {
  if (!boost::filesystem::exists(calib_result_file)) {
    std::ofstream outfile;
    outfile.open(calib_result_file, std::ios::app);
    outfile << "imu_topic,iteration_step，"
            << "p_IinL.x，p_IinL.y，p_IinL.z，"
            << "q_ItoL.x，q_ItoL.y，q_ItoL.z，q_ItoL.w，"
            << "time_offset，"
            << "g_refine.x，g_refine.y，gravity.z，"
            << "gyro_bias.x，gyro_bias.y，gyro_bias.z，"
            << "acce_bias.x，acce_bias.y，acce_bias.z"
            << "\n";
    outfile.close();
  }

  std::stringstream ss;
  ss << bag_name_ << "," << topic_imu_ << "," << iteration_num_;
  std::string dataset_info;
  ss >> dataset_info;

  std::string calib_info = calib_param_manager_->ParamString();

  std::ofstream outfile;
  outfile.open(calib_result_file, std::ios::app);
  outfile << dataset_info << "," << calib_info << std::endl;
  outfile.close();
}

void LICalibrHelper::SavePointCloud() const {
  if (0 == iteration_num_) {
    for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
      std::string surfel_path =
          cache_path_ + "/ndt_surfel_map-seg" + std::to_string(id) + ".pcd";
      surfel_association_vec_.at(id)->SaveSurfelsMap(surfel_path);
      //      lidar_odom_->SaveGlobalMap(cache_path_ + "/ndt_map.pcd");
    }
  } else {
    for (size_t id = 0; id < segment_dataset_->SegmentNum(); ++id) {
      std::string suffix =
          std::to_string(iteration_num_) + "-seg" + std::to_string(id) + ".pcd";

      std::string map_path = cache_path_ + "/refined_map-iter" + suffix;
      pcl::io::savePCDFileBinaryCompressed(
          map_path, *(scan_undistortion_vec_.at(id)->get_map_cloud()));
      std::cout << "Save refined map to " << map_path << "; size: "
                << scan_undistortion_vec_.at(id)->get_map_cloud()->size()
                << std::endl;

      std::string surfel_path = cache_path_ + "/surfel_map-iter" + suffix;
      surfel_association_vec_.at(id)->SaveSurfelsMap(surfel_path);
    }
  }
}
}  // namespace liso
