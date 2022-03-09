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

#ifndef TRAJECTORY_ESTIMATOR_H
#define TRAJECTORY_ESTIMATOR_H

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <factor/auto_diff/imu_factor.h>
#include <factor/auto_diff/lidar_feature_factor.h>
#include <factor/auto_diff/motion_factor.h>
#include <trajectory/se3_trajectory.h>
#include <utils/ceres_callbacks.h>
#include <basalt/spline/ceres_local_param.hpp>
#include <memory>
#include <thread>

#include <calib/surfel_association.h>
#include <sensor_data/calibration.h>
#include <sensor_data/imu_data.h>
#include <trajectory/trajectory_estimator_options.h>

namespace liso {

class TrajectoryEstimator {
  static ceres::Problem::Options DefaultProblemOptions() {
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }

 public:
  TrajectoryEstimator(std::shared_ptr<Trajectory> trajectory,
                      CalibParamManager::Ptr calib_param,
                      const TrajectoryEstimatorOptions& option)
      : trajectory_(trajectory), calib_param_(calib_param), options_(option) {
    problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());
    local_parameterization = new LieLocalParameterization<SO3d>();
  }

  void SetProblem(std::shared_ptr<ceres::Problem> problem_in) {
    problem_ = problem_in;
  }

  void SetTrajectory(std::shared_ptr<Trajectory> trajectory_in) {
    trajectory_ = trajectory_in;
  }

  void AddControlPoints(const SplineMeta<SplineOrder>& spline_meta,
                        std::vector<double*>& vec, bool addPosKont = false);

  void AddLoamMeasurement(const PointCorrespondence& pc, double weight,
                          double huber_loss = 5);

  void AddLiDARPoseMeasurement(const PoseData& pose_data, double rot_weight,
                               double pos_weight);

  void AddLiDARSurfelMeasurement(const PointCorrespondence& pc,
                                 double lidar_weight);

  void AddIMUGyroMeasurement(const IMUData& imu_data, double gyro_weight);

  void AddIMUMeasurement(const IMUData& imu_data, double gyro_weight,
                         double accel_weight);

  void AddPoseMeasurement(const PoseData& pose_data, double rot_weight,
                          double pos_weight);

  void AddPositionMeasurement(const PoseData& pose_data, double pos_weight);

  void AddOrientationMeasurement(const PoseData& pose_data, double rot_weight);

  void AddQuadraticIntegralFactor(double min_time, double max_time,
                                  Eigen::Matrix3d weight);

  void AddAngularVelocityConvexHullFactor(double time, double weight);

  void SetTrajectorControlPointVariable(double min_time, double max_time);

  /// Add callback for debug
  void AddCallback(bool needs_state = true, std::string filename = "");

  ceres::Solver::Summary Solve(int max_iterations = 50, bool progress = true,
                               int num_threads = -1);

  bool getCovariance();

 private:
  void LockIMUState(bool lock_ab, bool lock_wb, bool lock_g);

  void LockExtrinsicParam(bool lock_P, bool lock_R);

  CalibParamManager::Ptr calib_param_;

  TrajectoryEstimatorOptions options_;

  std::shared_ptr<Trajectory> trajectory_;
  std::shared_ptr<ceres::Problem> problem_;
  ceres::LocalParameterization* local_parameterization;

  bool callback_needs_state_;
  std::vector<std::unique_ptr<ceres::IterationCallback>> callbacks_;
};

}  // namespace liso

#endif
