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

namespace liso {

struct TrajectoryEstimatorOptions {
  // If we should optimize the trajectory
  bool lock_traj = false;

  // lock the extrinsic position/rotation/t_offset between imu and sensor
  bool lock_P = true;
  bool lock_R = true;
  bool lock_t_offset = true;

  // If estimating the time offset, the max/min value of time offset
  double t_offset_padding = 0.02;

  // lock the imu bias/gravity
  bool lock_ab = true;
  bool lock_wb = true;
  bool lock_g = true;

  bool lock_LiDAR_intrinsic = true;
  bool lock_IMU_intrinsic = true;
};

}  // namespace liso
