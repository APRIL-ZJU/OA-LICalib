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

#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace liso {

const double GRAVITY_NORM = -9.797;

using SO3d = Sophus::SO3<double>;
using SE3d = Sophus::SE3<double>;

struct IMUData {
  double timestamp;
  Eigen::Matrix<double, 3, 1> gyro;
  Eigen::Matrix<double, 3, 1> accel;
  Eigen::Quaterniond orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PoseData {
  PoseData()
      : timestamp(0),
        position(Eigen::Vector3d(0, 0, 0)),
        orientation(SO3d(Eigen::Quaterniond::Identity())) {}

  double timestamp;
  Eigen::Vector3d position;
  SO3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OdomData {
  double timestamp;
  Eigen::Matrix4d pose;
};

}  // namespace liso
