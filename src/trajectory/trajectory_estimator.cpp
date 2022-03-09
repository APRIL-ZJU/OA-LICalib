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

#include <trajectory/trajectory_estimator.h>

namespace liso {

void TrajectoryEstimator::AddControlPoints(
    const SplineMeta<SplineOrder>& spline_meta, std::vector<double*>& vec,
    bool addPosKont) {
  for (auto const& seg : spline_meta.segments) {
    size_t start_idx = trajectory_->computeTIndex(seg.t0 + 1e-9).second;
    for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i) {
      if (addPosKont) {
        vec.emplace_back(trajectory_->getKnotPos(i).data());
        problem_->AddParameterBlock(trajectory_->getKnotPos(i).data(), 3);
      } else {
        vec.emplace_back(trajectory_->getKnotSO3(i).data());
        problem_->AddParameterBlock(trajectory_->getKnotSO3(i).data(), 4,
                                    local_parameterization);
      }
      if (options_.lock_traj) {
        problem_->SetParameterBlockConstant(vec.back());
      }
    }
  }
}

void TrajectoryEstimator::AddLoamMeasurement(const PointCorrespondence& pc,
                                             double weight, double huber_loss) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta(
      {{pc.t_map, pc.t_map}, {pc.t_point, pc.t_point}}, spline_meta);

  using Functor = PointFeatureFactor;
  Functor* functor = new Functor(pc, spline_meta, weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }
  cost_function->AddParameterBlock(4);  // R_LtoI
  cost_function->AddParameterBlock(3);  // p_LinI

  cost_function->SetNumResiduals(1);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);

  vec.emplace_back(calib_param_->so3_LtoI.data());
  problem_->AddParameterBlock(calib_param_->so3_LtoI.data(), 4,
                              local_parameterization);
  vec.emplace_back(calib_param_->p_LinI.data());

  ceres::HuberLoss loss_function_(huber_loss);
  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddLiDARPoseMeasurement(const PoseData& pose_data,
                                                  double rot_weight,
                                                  double pos_weight) {
  SplineMeta<SplineOrder> spline_meta;
  if (options_.lock_t_offset) {
    double t_pose =
        pose_data.timestamp + trajectory_->GetTrajParam()->time_offset;
    if (t_pose < trajectory_->minTime() || t_pose >= trajectory_->maxTime())
      return;
    trajectory_->CaculateSplineMeta({{t_pose, t_pose}}, spline_meta);
  } else {
    double t_min = pose_data.timestamp - options_.t_offset_padding;
    double t_max = pose_data.timestamp + options_.t_offset_padding;
    if (t_min < trajectory_->minTime() || t_max >= trajectory_->maxTime())
      return;
    trajectory_->CaculateSplineMeta({{t_min, t_max}}, spline_meta);
  }

  using Functor = LiDARPoseFactor;
  Functor* functor =
      new Functor(pose_data, spline_meta, rot_weight, pos_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }
  cost_function->AddParameterBlock(4);  // R_LtoI
  cost_function->AddParameterBlock(3);  // p_LinI
  cost_function->AddParameterBlock(1);  // time_offset

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);
  vec.emplace_back(calib_param_->so3_LtoI.data());
  problem_->AddParameterBlock(calib_param_->so3_LtoI.data(), 4,
                              local_parameterization);
  vec.emplace_back(calib_param_->p_LinI.data());

  auto param = trajectory_->GetTrajParam();
  vec.emplace_back(&param->time_offset);

  problem_->AddParameterBlock(&param->time_offset, 1);
  if (options_.lock_t_offset) {
    problem_->SetParameterBlockConstant(&param->time_offset);
  } else {
    problem_->SetParameterLowerBound(&param->time_offset, 0,
                                     -options_.t_offset_padding);
    problem_->SetParameterUpperBound(&param->time_offset, 0,
                                     options_.t_offset_padding);
  }

  cost_function->SetNumResiduals(6);
  problem_->AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), vec);
}

void TrajectoryEstimator::AddLiDARSurfelMeasurement(
    const PointCorrespondence& pc, double lidar_weight) {
  SplineMeta<SplineOrder> spline_meta;

  if (options_.lock_t_offset) {
    double t_lidar = pc.t_point + trajectory_->GetTrajParam()->time_offset;
    if (t_lidar < trajectory_->minTime() || t_lidar >= trajectory_->maxTime())
      return;
    trajectory_->CaculateSplineMeta({{t_lidar, t_lidar}}, spline_meta);
  } else {
    double t_min = pc.t_point - options_.t_offset_padding;
    double t_max = pc.t_point + options_.t_offset_padding;
    if (t_min < trajectory_->minTime() || t_max >= trajectory_->maxTime())
      return;
    trajectory_->CaculateSplineMeta({{t_min, t_max}}, spline_meta);
  }

  using Functor = PointPlaneFactor;
  Functor* functor = new Functor(pc, spline_meta, lidar_weight,
                                 !options_.lock_LiDAR_intrinsic);

  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }
  cost_function->AddParameterBlock(4);  // R_LtoI
  cost_function->AddParameterBlock(3);  // p_LinI
  cost_function->AddParameterBlock(1);  // time_offset

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);

  vec.emplace_back(calib_param_->so3_LtoI.data());
  problem_->AddParameterBlock(calib_param_->so3_LtoI.data(), 4,
                              local_parameterization);

  vec.emplace_back(calib_param_->p_LinI.data());

  auto param = trajectory_->GetTrajParam();
  vec.emplace_back(&param->time_offset);

  problem_->AddParameterBlock(&param->time_offset, 1);
  if (options_.lock_t_offset) {
    problem_->SetParameterBlockConstant(&param->time_offset);
  } else {
    problem_->SetParameterLowerBound(&param->time_offset, 0,
                                     -options_.t_offset_padding);
    problem_->SetParameterUpperBound(&param->time_offset, 0,
                                     options_.t_offset_padding);
  }

  if (!options_.lock_LiDAR_intrinsic) {
    /// opt_laser_param
    int laser_id = int(pc.point_raw.x());
    for (int idx = 0; idx < 6; ++idx) {
      cost_function->AddParameterBlock(1);  // laser_param
      auto laser_param =
          calib_param_->lidar_intrinsic.GetLaserParam(laser_id, idx);
      vec.emplace_back(laser_param);

      problem_->AddParameterBlock(laser_param, 1);

      // fix fist laser_id' param partly
      if (0 == laser_id && idx > 1) {
        problem_->SetParameterBlockConstant(laser_param);
      }
      // horiz_offset_mm
      //      if (idx == 3) {
      //        problem_->SetParameterBlockConstant(laser_param);
      //      }
      if (5 == idx) {
        problem_->SetParameterLowerBound(laser_param, 0, -0.017);  // 1 degree
        problem_->SetParameterUpperBound(laser_param, 0, 0.017);
      }
    }
  }

  ceres::LossFunction* loss_function;
  loss_function = new ceres::CauchyLoss(1.0);  // adopt from vins-mono
  cost_function->SetNumResiduals(1);
  // problem_->AddResidualBlock(cost_function, NULL, vec);
  problem_->AddResidualBlock(cost_function, loss_function, vec);
}

void TrajectoryEstimator::AddIMUMeasurement(const IMUData& imu_data,
                                            double gyro_weight,
                                            double accel_weight) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{imu_data.timestamp, imu_data.timestamp}},
                                  spline_meta);
  using Functor = GyroAcceWithConstantBiasFactor;
  Functor* functor =
      new Functor(imu_data, spline_meta, gyro_weight, accel_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }
  cost_function->AddParameterBlock(3);  // gyro bias
  cost_function->AddParameterBlock(3);  // acce bias
  cost_function->AddParameterBlock(2);  // g_refine

  cost_function->SetNumResiduals(6);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);

  auto param = trajectory_->GetTrajParam();
  vec.emplace_back(param->gyro_bias.data());
  vec.emplace_back(param->acce_bias.data());
  vec.emplace_back(param->g_refine.data());

  problem_->AddParameterBlock(param->g_refine.data(), 2);
  // for (int i = 0; i < 2; i++) {
  //   problem_->SetParameterLowerBound(param->g_refine.data(), i, -M_PI / 2.0);
  //   problem_->SetParameterUpperBound(param->g_refine.data(), i, M_PI / 2.0);
  // }

  /// opt_imu_param
  {
    cost_function->AddParameterBlock(6);  // M_w
    cost_function->AddParameterBlock(6);  // M_a
    cost_function->AddParameterBlock(9);  // A_w
    cost_function->AddParameterBlock(4);  // q_WtoA

    auto M_w_param = calib_param_->imu_intrinsic.GetMwVector();
    auto M_a_param = calib_param_->imu_intrinsic.GetMaVector();
    auto A_w_param = calib_param_->imu_intrinsic.GetAwVector();
    auto q_WtoA_param = calib_param_->imu_intrinsic.GetQWtoAVector();
    vec.emplace_back(M_w_param);
    vec.emplace_back(M_a_param);
    vec.emplace_back(A_w_param);
    vec.emplace_back(q_WtoA_param);

    problem_->AddParameterBlock(M_w_param, 6);
    problem_->AddParameterBlock(M_a_param, 6);
    problem_->AddParameterBlock(A_w_param, 9);
    problem_->AddParameterBlock(q_WtoA_param, 4, local_parameterization);

    if (options_.lock_IMU_intrinsic) {
      problem_->SetParameterBlockConstant(M_w_param);
      problem_->SetParameterBlockConstant(M_a_param);
      problem_->SetParameterBlockConstant(A_w_param);
      problem_->SetParameterBlockConstant(q_WtoA_param);
    } else {
      // 不考虑 G sensitivity
      problem_->SetParameterBlockConstant(A_w_param);
    }
    if (false) {
      // Subset parametrization fix partly param in SubsetParameterization
      /// 0 3 6
      /// 1 4 7
      /// 2 5 8
      static std::vector<int> vec_constant_param = {1, 2, 5};
      // vec_constant_param.insert(vec_constant_param.end(), {1,2,5});
      static ceres::SubsetParameterization* subset_parameterization =
          new ceres::SubsetParameterization(9, vec_constant_param);
      // problem_->AddParameterBlock(A_w_param, 9, subset_parameterization);
      problem_->AddParameterBlock(A_w_param, 9);
      problem_->SetParameterization(A_w_param, subset_parameterization);
    }
  }

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddIMUGyroMeasurement(const IMUData& imu_data,
                                                double gyro_weight) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{imu_data.timestamp, imu_data.timestamp}},
                                  spline_meta);
  using Functor = GyroscopeWithConstantBiasFactor;
  Functor* functor = new Functor(imu_data, spline_meta, gyro_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  cost_function->AddParameterBlock(3);  // gyro bias

  cost_function->SetNumResiduals(3);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);

  auto param = trajectory_->GetTrajParam();
  vec.emplace_back(param->gyro_bias.data());
  problem_->AddParameterBlock(param->gyro_bias.data(), 3);
  problem_->SetParameterBlockConstant(param->gyro_bias.data());

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddPoseMeasurement(const PoseData& pose_data,
                                             double rot_weight,
                                             double pos_weight) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{pose_data.timestamp, pose_data.timestamp}},
                                  spline_meta);

  using Functor = IMUPoseFactor;
  Functor* functor =
      new Functor(pose_data, spline_meta, rot_weight, pos_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }

  cost_function->SetNumResiduals(6);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddPositionMeasurement(const PoseData& pose_data,
                                                 double pos_weight) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{pose_data.timestamp, pose_data.timestamp}},
                                  spline_meta);

  using Functor = IMUPositionFactor;
  Functor* functor = new Functor(pose_data, spline_meta, pos_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }

  cost_function->SetNumResiduals(3);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec, true);

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddOrientationMeasurement(const PoseData& pose_data,
                                                    double rot_weight) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{pose_data.timestamp, pose_data.timestamp}},
                                  spline_meta);

  using Functor = IMUOrientationFactor;
  Functor* functor = new Functor(pose_data, spline_meta, rot_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add SO3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }

  cost_function->SetNumResiduals(3);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddQuadraticIntegralFactor(double min_time,
                                                     double max_time,
                                                     Eigen::Matrix3d weight) {
  size_t min_s = trajectory_->computeTIndex(min_time).second;
  size_t max_s = trajectory_->computeTIndex(max_time).second;

  std::vector<double> times;
  for (size_t i = min_s; i <= max_s; i++) {
    double timestamp = trajectory_->minTime() + i * trajectory_->getDt();
    times.push_back(timestamp);
  }
  times.back() += 1e-10;
  std::cout << YELLOW << "[AddQuadraticIntegralFactor] [" << std::fixed
            << std::setprecision(5)
            << trajectory_->minTime() + min_s * trajectory_->getDt() << ", "
            << trajectory_->minTime() + max_s * trajectory_->getDt() << "]\n"
            << RESET;

  if (times.empty()) return;

  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{times.front(), times.back()}}, spline_meta);

  using Functor = QuadraticIntegralFactor<SplineOrder, 2>;
  Functor* functor = new Functor(times, spline_meta, weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add R3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }

  cost_function->SetNumResiduals(3 * SplineOrder * times.size());

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec, true);

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::AddAngularVelocityConvexHullFactor(double time,
                                                             double weight) {
  SplineMeta<SplineOrder> spline_meta;
  trajectory_->CaculateSplineMeta({{time, time}}, spline_meta);

  using Functor = AngularVelocityConvexHullFactor;
  Functor* functor = new Functor(spline_meta, weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }

  size_t v_kont_num = spline_meta.NumParameters() - spline_meta.segments.size();
  cost_function->SetNumResiduals(v_kont_num);

  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);

  problem_->AddResidualBlock(cost_function, NULL, vec);
}

void TrajectoryEstimator::SetTrajectorControlPointVariable(double min_time,
                                                           double max_time) {
  size_t min_s = trajectory_->computeTIndex(min_time).second;
  size_t max_s = trajectory_->computeTIndex(max_time).second;

  std::cout << "[SetTrajectorControlPointVariable]: " << min_s << ", "
            << max_s + SplineOrder - 1 << "\n";
  for (size_t i = min_s; i < (max_s + SplineOrder); i++) {
    problem_->AddParameterBlock(trajectory_->getKnotSO3(i).data(), 4,
                                local_parameterization);
    problem_->SetParameterBlockVariable(trajectory_->getKnotSO3(i).data());

    problem_->AddParameterBlock(trajectory_->getKnotPos(i).data(), 3);
    problem_->SetParameterBlockVariable(trajectory_->getKnotPos(i).data());
  }
}

/// Add callback for debug
void TrajectoryEstimator::AddCallback(bool needs_state, std::string filename) {
  std::unique_ptr<CheckStateCallback> cb =
      std::make_unique<CheckStateCallback>(filename);

  cb->addCheckState("px py pz", 3, calib_param_->p_LinI.data());
  cb->addCheckState("qx qy qz qw", 4, calib_param_->so3_LtoI.data());

  for (SegmentCalibParam& v : calib_param_->segment_param) {
    if (problem_->HasParameterBlock(v.acce_bias.data())) {
      cb->addCheckState("ax ay az", 3, v.acce_bias.data());
    }
    if (problem_->HasParameterBlock(v.gyro_bias.data())) {
      cb->addCheckState("gx gy gz", 3, v.gyro_bias.data());
    }
    if (problem_->HasParameterBlock(v.g_refine.data())) {
      cb->addCheckState("g_refinex g_refiney", 2, v.g_refine.data());
    }
    if (problem_->HasParameterBlock(&v.time_offset)) {
      cb->addCheckState("time_offset", 1, &v.time_offset);
    }
  }
  callbacks_.push_back(std::move(cb));

  // If any callback requires state, the flag must be set
  if (!callback_needs_state_) callback_needs_state_ = needs_state;
}

void TrajectoryEstimator::LockIMUState(bool lock_ab, bool lock_wb,
                                       bool lock_g) {
  for (SegmentCalibParam& v : calib_param_->segment_param) {
    if (lock_ab && problem_->HasParameterBlock(v.acce_bias.data())) {
      problem_->AddParameterBlock(v.acce_bias.data(), 3);
      problem_->SetParameterBlockConstant(v.acce_bias.data());
    }
    if (lock_wb && problem_->HasParameterBlock(v.gyro_bias.data())) {
      problem_->AddParameterBlock(v.gyro_bias.data(), 3);
      problem_->SetParameterBlockConstant(v.gyro_bias.data());
    }
    if (lock_g && problem_->HasParameterBlock(v.g_refine.data())) {
      problem_->AddParameterBlock(v.g_refine.data(), 2);
      problem_->SetParameterBlockConstant(v.g_refine.data());
    }
  }
}

void TrajectoryEstimator::LockExtrinsicParam(bool lock_P, bool lock_R) {
  if (lock_P) {
    problem_->AddParameterBlock(calib_param_->p_LinI.data(), 3);
    problem_->SetParameterBlockConstant(calib_param_->p_LinI.data());
  }
  if (lock_R) {
    problem_->AddParameterBlock(calib_param_->so3_LtoI.data(), 4,
                                local_parameterization);
    problem_->SetParameterBlockConstant(calib_param_->so3_LtoI.data());
  }
}

ceres::Solver::Summary TrajectoryEstimator::Solve(int max_iterations,
                                                  bool progress,
                                                  int num_threads) {
  LockIMUState(options_.lock_ab, options_.lock_wb, options_.lock_g);

  LockExtrinsicParam(options_.lock_P, options_.lock_R);

  ceres::Solver::Options options;

  options.minimizer_type = ceres::TRUST_REGION;
  //  options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  //  options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //    options.trust_region_strategy_type = ceres::DOGLEG;
  //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  //    options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  if (!options_.lock_P) {
    // function_tolerance = 1e-6;
    // gradient_tolerance = 1e-10;
    // parameter_tolerance = 1e-8;
    // options.function_tolerance = 1e-20;
    // options.parameter_tolerance = 1e-20;
    // options.initial_trust_region_radius = 1e3;
    max_iterations = 100;
  }

  options.minimizer_progress_to_stdout = progress;

  if (num_threads < 1) {
    num_threads = std::thread::hardware_concurrency();
  }
  options.num_threads = num_threads;
  options.max_num_iterations = max_iterations;

  if (callbacks_.size() > 0) {
    for (auto& cb : callbacks_) {
      options.callbacks.push_back(cb.get());
    }

    if (callback_needs_state_) options.update_state_every_iteration = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  // update state
  calib_param_->UpdateExtrinicParam();
  calib_param_->UpdateGravity();
  //  getCovariance();
  return summary;
}

bool TrajectoryEstimator::getCovariance() {
  ceres::Covariance::Options options;
  // options.algorithm_type = ceres::DENSE_SVD;
  options.apply_loss_function = false;
  ceres::Covariance covariance(options);

  if (!problem_->HasParameterBlock(calib_param_->p_LinI.data()) ||
      !problem_->HasParameterBlock(calib_param_->so3_LtoI.data())) {
    return false;
  }

  std::vector<const double*> vec;
  vec.emplace_back(calib_param_->p_LinI.data());
  vec.emplace_back(calib_param_->so3_LtoI.data());

  if (!covariance.Compute(vec, problem_.get())) {
    std::cout
        << "[CovarianceMatrixInTangentSpace] J^TJ is a rank deficient matrix\n";
    return false;
  }

  double m2cm = 100;
  double rad2degree = 180.0 / M_PI;

  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  covariance.GetCovarianceMatrixInTangentSpace(vec, cov.data());

  Eigen::VectorXd diag;
  diag = cov.diagonal();
  diag.head<6>() = diag.head<6>().cwiseSqrt();
  diag.segment<3>(0) *= m2cm;
  diag.segment<3>(3) *= rad2degree;

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "[CovarianceMatrixInTangentSpace] \n" << cov << std::endl;
  std::cout << YELLOW;
  std::cout << "[std] pos (cm)    : " << diag.segment<3>(0).transpose()
            << std::endl;
  std::cout << "[std] rot (degree): " << diag.segment<3>(3).transpose() << RESET
            << std::endl;
  return true;
}

}  // namespace liso
