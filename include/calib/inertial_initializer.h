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

#ifndef INERTIAL_INITIALIZER_H
#define INERTIAL_INITIALIZER_H

#include <sensor_data/cloud_type.h>
#include <utils/math_utils.h>
#include <trajectory/se3_trajectory.h>

namespace liso {

class InertialInitializer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<InertialInitializer> Ptr;

  explicit InertialInitializer()
      : rotaion_initialized_(false),
        q_ItoS_est_(Eigen::Quaterniond::Identity()) {}

  bool BuildProblem(std::shared_ptr<Trajectory> p_trajectory,
                    const Eigen::aligned_vector<OdomData>& odom_data,
                    Eigen::MatrixXd& A);

  bool EstimateRotation(std::shared_ptr<Trajectory> p_trajectory,
                        const Eigen::aligned_vector<OdomData>& odom_data);

  bool SolveConstraintqyx(const Eigen::Vector4d t1, const Eigen::Vector4d t2,
                          double& x1, double& x2);

  bool EstimateRotationRyx(std::shared_ptr<Trajectory> p_trajectory,
                           const Eigen::aligned_vector<OdomData>& odom_data);

  bool isInitialized() { return rotaion_initialized_; }

  Eigen::Quaterniond getQ_ItoS() { return q_ItoS_est_; }

 private:
  bool rotaion_initialized_;
  Eigen::Quaterniond q_ItoS_est_;
};

}  // namespace liso

#endif
