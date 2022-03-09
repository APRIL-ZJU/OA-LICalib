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

#ifndef CALIB_TOOL_H
#define CALIB_TOOL_H

#include <calib/inertial_initializer.h>
#include <calib/io/dataset_reader.h>
#include <calib/lidar_localization.h>
#include <calib/lidar_ndt_odometry.h>
#include <calib/scan_undistortion.h>
#include <calib/surfel_association.h>
#include <trajectory/trajectory_estimator.h>
#include <utils/tic_toc.h>

#include <trajectory/trajectory_viewer.h>

namespace liso {

namespace CalibTool {

inline void InitialSO3TrajWithGyro(
    const CalibParamManager::Ptr calib_param,
    const Eigen::aligned_vector<IMUData>& imu_data,
    std::shared_ptr<Trajectory> trajectory) {
  double gyro_weight = 1.0;
  double orientation_weight = 1.0;

  TrajectoryEstimatorOptions options;
  TrajectoryEstimator estimator(trajectory, calib_param, options);

  for (auto const& v : imu_data) {
    estimator.AddIMUGyroMeasurement(v, gyro_weight);
  }

  /// Fix origin orientation;
  PoseData origin_pose;
  origin_pose.timestamp = imu_data.front().timestamp;
  estimator.AddOrientationMeasurement(origin_pose, orientation_weight);

  ceres::Solver::Summary summary = estimator.Solve(50, false);
  std::cout << "[InitialSO3TrajWithGyro]: \n"
            << summary.BriefReport() << std::endl;
}

inline void InitialIMUTrajectory(
    const CalibParamManager::Ptr calib_param,
    const Eigen::aligned_vector<OdomData>& odom_data,
    std::shared_ptr<Trajectory> imu_trajectory) {
  std::cout << "[InitialIMUTrajectory]\n";
  std::cout << "odom_data t: " << odom_data.back().timestamp << std::endl;
  std::cout << "traj t: " << imu_trajectory->maxTime() << std::endl;

  auto raw_odom_traj =
      std::make_shared<Trajectory>(0.2, imu_trajectory->minTime());
  raw_odom_traj->extendKnotsTo(imu_trajectory->maxTime(),
                               SO3d(Eigen::Quaterniond::Identity()),
                               Eigen::Vector3d(0, 0, 0));
  raw_odom_traj->SetCalibParam(calib_param);

  Sophus::SO3d R_ItoL = Sophus::SO3d(calib_param->q_LtoI.conjugate());
  Eigen::Vector3d p_IinL = R_ItoL * (-calib_param->p_LinI);

  TrajectoryEstimatorOptions options;
  TrajectoryEstimator estimator(raw_odom_traj, calib_param, options);
  for (auto const& v : odom_data) {
    Sophus::SO3d R_MtoL =
        Sophus::SO3d(Eigen::Quaterniond(v.pose.block<3, 3>(0, 0)));

    PoseData pose_ItoM;
    pose_ItoM.timestamp = v.timestamp;
    pose_ItoM.orientation = R_MtoL * R_ItoL;
    pose_ItoM.position =
        v.pose.block<3, 3>(0, 0) * p_IinL + v.pose.block<3, 1>(0, 3);
    estimator.AddPoseMeasurement(pose_ItoM, 1, 1);
  }
  {
    auto const& v = odom_data.front();
    Sophus::SO3d R_MtoL =
        Sophus::SO3d(Eigen::Quaterniond(v.pose.block<3, 3>(0, 0)));
    PoseData pose_ItoM;
    pose_ItoM.timestamp = raw_odom_traj->minTime() + 1e-9;
    pose_ItoM.orientation = R_MtoL * R_ItoL;
    pose_ItoM.position =
        v.pose.block<3, 3>(0, 0) * p_IinL + v.pose.block<3, 1>(0, 3);
    estimator.AddPoseMeasurement(pose_ItoM, 0.3, 0.3);
  }

  {
    auto const& v = odom_data.back();
    Sophus::SO3d R_MtoL =
        Sophus::SO3d(Eigen::Quaterniond(v.pose.block<3, 3>(0, 0)));
    PoseData pose_ItoM;
    pose_ItoM.timestamp = raw_odom_traj->maxTime() - 1e-9;
    pose_ItoM.orientation = R_MtoL * R_ItoL;
    pose_ItoM.position =
        v.pose.block<3, 3>(0, 0) * p_IinL + v.pose.block<3, 1>(0, 3);
    estimator.AddPoseMeasurement(pose_ItoM, 0.3, 0.3);
  }

  ceres::Solver::Summary summary = estimator.Solve(50, false);
  std::cout << summary.BriefReport() << std::endl;
  if (!summary.IsSolutionUsable()) {
    std::cerr << "[InitialIMUTrajectory]: NOT convergence \n"
              << summary.BriefReport() << std::endl;
  }

  std::cout << "Refine trajectory" << std::endl;
  std::cout << "imu_trajectory t: " << imu_trajectory->maxTime() << std::endl;
  std::cout << "raw_odom_traj t: " << raw_odom_traj->maxTime() << std::endl;

  /// Refine trajectory
  double sample_time = imu_trajectory->getDt() * 0.43;
  TrajectoryEstimator odom_estimator(imu_trajectory, calib_param, options);
  for (double t = imu_trajectory->minTime() + sample_time;
       t < imu_trajectory->maxTime(); t += sample_time) {
    PoseData traj_pose;
    traj_pose.timestamp = t;
    traj_pose.position = raw_odom_traj->pose(t).translation();
    traj_pose.orientation = raw_odom_traj->pose(t).so3();
    odom_estimator.AddPoseMeasurement(traj_pose, 1, 1);
  }
  {
    double t = imu_trajectory->maxTime() - 1e-9;
    PoseData traj_pose;
    traj_pose.timestamp = t;
    traj_pose.position = raw_odom_traj->pose(t).translation();
    traj_pose.orientation = raw_odom_traj->pose(t).so3();
    odom_estimator.AddPoseMeasurement(traj_pose, 1, 1);
  }

  ceres::Solver::Summary odom_summary = odom_estimator.Solve(50, false);
  if (!summary.IsSolutionUsable()) {
    std::cerr << "[InitialIMUTrajectory]: NOT convergence \n"
              << odom_summary.BriefReport() << std::endl;
  }

  //    TrajectoryViewer::PublishSplineTrajectory(
  //            imu_trajectory,
  //            imu_trajectory->minTime(),
  //            imu_trajectory->maxTime(), 0.02);
  //      TrajectoryViewer::PublishViconData(imu_trajectory, imu_init_data);
}

inline void GetLidarOdometry(
    const std::vector<double>& scan_timestamps,
    const std::map<double, LiDARFeature>& undistort_scan_data,
    const std::shared_ptr<Trajectory>& trajectory,
    LidarNdtOdometry::Ptr lidar_odom, double scan4map_time = -1) {
  bool update_map = true;
  double last_scan_time = 0;
  for (const double& scan_time : scan_timestamps) {
    if (scan4map_time > 0 && scan_time > scan4map_time) update_map = false;
    auto iter = undistort_scan_data.find(scan_time);
    if (iter != undistort_scan_data.end()) {
      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_L2toL1 = Eigen::Quaterniond::Identity();
      if (last_scan_time > 0 && trajectory->EvaluateLidarRelativeRotation(
                                    last_scan_time, scan_time, q_L2toL1)) {
        pose_predict.block<3, 3>(0, 0) = q_L2toL1.toRotationMatrix();
      }
      lidar_odom->FeedScan(iter->second, pose_predict, update_map);
      last_scan_time = scan_time;
    }
  }
}

inline void GetLidarLocatorResult(
    const std::vector<double>& scan_timestamps,
    const std::map<double, LiDARFeature>& undistort_scan_data,
    const std::shared_ptr<Trajectory>& trajectory,
    LIDARLocalization::Ptr& ndt_locator) {
  double last_scan_time = 0;
  for (const double& scan_time : scan_timestamps) {
    auto iter = undistort_scan_data.find(scan_time);
    if (iter != undistort_scan_data.end()) {
      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_L2toL1 = Eigen::Quaterniond::Identity();
      if (last_scan_time > 0 && trajectory->EvaluateLidarRelativeRotation(
                                    last_scan_time, scan_time, q_L2toL1)) {
        pose_predict.block<3, 3>(0, 0) = q_L2toL1.toRotationMatrix();
      }

      ndt_locator->FeedScan(iter->second, pose_predict);
      ndt_locator->PublishCloudAndOdom(iter->second.full_features);
      last_scan_time = scan_time;
    }
  }
  ndt_locator->CaculateGlobalMapAndOdom(undistort_scan_data);
}

inline bool EstimateRotation(const Eigen::aligned_vector<IMUData>& imu_data,
                             const std::vector<LiDARFeature>& scan_data,
                             std::shared_ptr<Trajectory> trajectory,
                             CalibParamManager::Ptr calib_param) {
  LidarNdtOdometry lidar_odom(calib_param->lo_param.ndt_resolution,
                              calib_param->lo_param.ndt_key_frame_downsample);

  InertialInitializer rot_initer;
  for (auto const& scan_raw : scan_data) {
    lidar_odom.FeedScan(scan_raw);
    if (lidar_odom.get_odom_data().size() < 20 ||
        (lidar_odom.get_odom_data().size() % 5 != 0))
      continue;

    bool ret;
    ret = rot_initer.EstimateRotation(trajectory, lidar_odom.get_odom_data());

    if (ret) {
      Eigen::Quaterniond qItoL = rot_initer.getQ_ItoS();
      calib_param->so3_LtoI.setQuaternion(qItoL.conjugate());
      calib_param->UpdateExtrinicParam();
      Eigen::Vector3d euler_ItoL =
          qItoL.toRotationMatrix().eulerAngles(0, 1, 2);
      std::cout << "[Initialization] Done. Euler_ItoL initial degree: "
                << (euler_ItoL * 180.0 / M_PI).transpose() << std::endl;
      return true;
    }
  }

  return false;
}

inline bool DataAssociation(
    const std::vector<LiDARFeature>& scan_data,
    const std::map<double, LiDARFeature>& scan_data_in_map,
    SurfelAssociation::Ptr surfel_association) {
  /// get association
  for (auto const& scan_raw : scan_data) {
    auto iter = scan_data_in_map.find(scan_raw.timestamp);
    if (iter == scan_data_in_map.end()) {
      continue;
    }
    surfel_association->GetAssociation(iter->second.full_features,
                                       scan_raw.full_features,
                                       scan_raw.raw_data, 2);
  }

  surfel_association->AverageTimeDownSmaple();
  // surfel_association->randomDownSample(10);

  std::cout << "Surfel point number: "
            << surfel_association->get_surfel_points().size() << std::endl;

  if (surfel_association->get_surfel_points().size() > 10) {
    return true;
  } else {
    std::cerr << "[DataAssociation] fails.\n";
    return false;
  }
}

inline pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr GetNDtPtr(
    const PosCloud::Ptr& map_cloud, double ndt_resolution) {
  auto ndt_omp = pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr(
      new pclomp::NormalDistributionsTransform<PosPoint, PosPoint>());
  ndt_omp->setResolution(ndt_resolution);
  ndt_omp->setNumThreads(4);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setTransformationEpsilon(1e-3);
  ndt_omp->setStepSize(0.01);
  ndt_omp->setMaximumIterations(50);

  ndt_omp->setInputTarget(map_cloud);

  return ndt_omp;
}

inline bool DataAssociationWithOdom(
    const std::vector<LiDARFeature>& scan_data,
    const std::vector<double>& scan_timestamps,
    const std::pair<double, double>& segment_timestamp,
    const std::shared_ptr<Trajectory> trajectory,
    const CalibParamManager::Ptr calib_param,
    std::shared_ptr<ScanUndistortion> scan_undistortion,
    SurfelAssociation::Ptr surfel_association, const std::string& cache_path) {
  TicToc timer;

  timer.tic();
  scan_undistortion->UndistortScan(trajectory, scan_data, false);

  auto lo_param = &calib_param->lo_param;
  LidarNdtOdometry::Ptr lidar_odom = std::make_shared<LidarNdtOdometry>(
      lo_param->ndt_resolution, lo_param->ndt_key_frame_downsample);

  GetLidarOdometry(scan_timestamps, scan_undistortion->get_scan_data(),
                   trajectory, lidar_odom);
  std::cout << "[Paper] rerun_odom costs " << timer.toc() << " ms\n";

  lidar_odom->SaveGlobalMap(cache_path + "/ndt_map.pcd");

  double relative_start_time = segment_timestamp.first;
  double relative_end_time = segment_timestamp.second;
  lidar_odom->OdomToTUMTxt(cache_path, relative_start_time, relative_end_time);

  timer.tic();
  CalibTool::InitialIMUTrajectory(calib_param, lidar_odom->get_odom_data(),
                                  trajectory);
  std::cout << "[Paper] InitialIMUTrajectory costs " << timer.toc() << " ms\n";

  timer.tic();
  scan_undistortion->UndistortScanInMap(lidar_odom->get_odom_data());

  surfel_association->SetSurfelMap(lidar_odom->get_ndt_ptr(),
                                   scan_data.front().timestamp);

  bool ret = DataAssociation(
      scan_data, scan_undistortion->get_scan_data_in_map(), surfel_association);

  std::cout << "[Paper] DataAssociation costs " << timer.toc() << " ms\n";
  return ret;
}

inline bool DataAssociationWithLocator(
    const std::vector<LiDARFeature>& scan_data,
    const std::vector<double>& scan_timestamps,
    const std::pair<double, double>& segment_timestamp,
    const std::shared_ptr<Trajectory> trajectory,
    const CalibParamManager::Ptr calib_param,
    LIDARLocalization::Ptr& ndt_locator,
    std::shared_ptr<ScanUndistortion> scan_undistortion,
    SurfelAssociation::Ptr surfel_association, const std::string& cache_path) {
  scan_undistortion->UndistortScan(trajectory, scan_data, false);

  GetLidarLocatorResult(scan_timestamps, scan_undistortion->get_scan_data(),
                        trajectory, ndt_locator);
  ndt_locator->SaveGlobalMap(cache_path + "/ndt_locator_map.pcd");

  double relative_start_time = segment_timestamp.first;
  double relative_end_time = segment_timestamp.second;
  ndt_locator->OdomToTUMTxt(cache_path, relative_start_time, relative_end_time);

  CalibTool::InitialIMUTrajectory(calib_param, ndt_locator->get_odom_data(),
                                  trajectory);

  scan_undistortion->UndistortScanInMap(ndt_locator->get_odom_data());

  auto ndt_omp = GetNDtPtr(ndt_locator->GetNewMapCloud(),
                           calib_param->lo_param.ndt_resolution);
  surfel_association->SetSurfelMap(ndt_omp, scan_data.front().timestamp);

  bool ret = DataAssociation(
      scan_data, scan_undistortion->get_scan_data_in_map(), surfel_association);
  return ret;
}

inline bool DataAssociationWithTraj(
    const std::vector<LiDARFeature>& scan_data,
    const std::shared_ptr<Trajectory> trajectory,
    const CalibParamManager::Ptr calib_param,
    std::shared_ptr<ScanUndistortion> scan_undistortion,
    SurfelAssociation::Ptr surfel_association) {
  scan_undistortion->UndistortScanInMap(trajectory, scan_data, true);
  std::cout << "get refined map done\n";

  double plane_lambda = 0.6;
  surfel_association->SetPlaneLambda(plane_lambda);

  auto ndt_omp = GetNDtPtr(scan_undistortion->get_map_cloud(),
                           calib_param->lo_param.ndt_resolution);
  surfel_association->SetSurfelMap(ndt_omp, scan_data.front().timestamp);

  bool ret = DataAssociation(
      scan_data, scan_undistortion->get_scan_data_in_map(), surfel_association);
  return ret;
}

inline void GetLidarPointCorrespondence(
    const SurfelAssociation::Ptr surfel_association,
    std::pair<double, double> selected_time,
    Eigen::aligned_vector<PointCorrespondence>& point_measurement,
    std::pair<double, double>& valid_time) {
  double valid_time_min = selected_time.second;
  double valid_time_max = selected_time.first;

  for (SurfelPoint sp : surfel_association->get_surfel_points()) {
    if (sp.timestamp < selected_time.first ||
        sp.timestamp >= selected_time.second)
      continue;
    PointCorrespondence pc;
    pc.t_map = surfel_association->get_maptime();
    pc.t_point = sp.timestamp;
    pc.point = sp.point;
    pc.point_raw = sp.point_raw;
    pc.geo_type = GeometryType::Plane;
    pc.geo_plane = surfel_association->get_surfel_planes().at(sp.plane_id).p4;

    point_measurement.emplace_back(pc);

    valid_time_min = pc.t_point < valid_time_min ? pc.t_point : valid_time_min;
    valid_time_max = pc.t_point > valid_time_max ? pc.t_point : valid_time_max;
  }
  valid_time.first = valid_time_min;
  valid_time.second = valid_time_max;
}

inline Eigen::Vector3d GetLidarCov(
    const Eigen::aligned_vector<PointCorrespondence>& point_measurement) {
  Eigen::Matrix3d nnt = Eigen::Matrix3d::Zero();
  for (PointCorrespondence pc : point_measurement) {
    nnt += pc.geo_plane.head(3) * pc.geo_plane.head(3).transpose();
  }

  if (!point_measurement.empty()) {
    nnt /= point_measurement.size();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        nnt, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d plane_cov = svd.singularValues();
    return plane_cov;
  } else {
    return Eigen::Vector3d::Zero();
  }
}

inline void AddIMUAndSurfelToProblem(
    const CalibWeights& calib_weights,
    const Eigen::aligned_vector<IMUData> imu_data,
    const Eigen::aligned_vector<PointCorrespondence>& point_measurement,
    const std::pair<double, double> lidar_valid_time,
    std::shared_ptr<Trajectory> trajectory,
    std::shared_ptr<TrajectoryEstimator> estimator) {
  double gyro_weight = calib_weights.opt_gyro_weight;
  double acce_weight = calib_weights.opt_acce_weight;
  double lidar_weight = calib_weights.opt_lidar_weight;

  /// [step1] Add IMU measurement
  for (const IMUData& data : imu_data) {
    if (trajectory->GetTrajQuality(data.timestamp)) {
      estimator->AddIMUMeasurement(data, gyro_weight, acce_weight);
    }
  }

  /// [step2] Add surfel measurement
  for (const PointCorrespondence& pc : point_measurement) {
    estimator->AddLiDARSurfelMeasurement(pc, lidar_weight);
  }

  // estimator->AddCallback(true);

  std::pair<double, double> valid_time = lidar_valid_time;
  if (imu_data.front().timestamp < valid_time.first)
    valid_time.first = imu_data.front().timestamp;
  if (imu_data.back().timestamp > valid_time.second)
    valid_time.second = imu_data.back().timestamp;
}

}  // namespace CalibTool
}  // namespace liso

#endif
