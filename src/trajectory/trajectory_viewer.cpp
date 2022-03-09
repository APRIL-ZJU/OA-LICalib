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

#include <trajectory/trajectory_viewer.h>

namespace liso {

namespace publisher {
ros::Publisher pub_trajectory_raw_;
ros::Publisher pub_trajectory_est_;
ros::Publisher pub_imu_raw_array_;
ros::Publisher pub_imu_est_array_;
ros::Publisher pub_target_cloud_;
ros::Publisher pub_source_cloud_;

ros::Publisher pub_spline_trajectory_;
ros::Publisher pub_lidar_trajectory_;

void SetPublisher(ros::NodeHandle &nh) {
  /// Vicon data
  pub_trajectory_raw_ = nh.advertise<oa_licalib::pose_array>("/path_raw", 10);
  pub_trajectory_est_ = nh.advertise<oa_licalib::pose_array>("/path_est", 10);
  /// IMU fitting results
  pub_imu_raw_array_ = nh.advertise<oa_licalib::imu_array>("/imu_raw_array", 10);
  pub_imu_est_array_ = nh.advertise<oa_licalib::imu_array>("/imu_est_array", 10);
  /// lidar matching results
  pub_target_cloud_ =
      nh.advertise<sensor_msgs::PointCloud2>("/target_cloud", 10);
  pub_source_cloud_ =
      nh.advertise<sensor_msgs::PointCloud2>("/source_cloud", 10);

  /// spline trajectory
  pub_spline_trajectory_ =
      nh.advertise<nav_msgs::Path>("/spline_trajectory", 10);

  /// spline trajectory
  pub_lidar_trajectory_ = nh.advertise<nav_msgs::Path>("/lidar_trajectory", 10);
}

}  // namespace publisher

}  // namespace liso
