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

#ifndef SCAN_UNDISTORTION_H
#define SCAN_UNDISTORTION_H

#include <calib/io/dataset_reader.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_data/cloud_type.h>
#include <sensor_data/lidar_feature.h>
#include <trajectory/se3_trajectory.h>

namespace liso {

class ScanUndistortion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ScanUndistortion() {}

  void UndistortScan(std::shared_ptr<Trajectory> trajectory,
                     const std::vector<LiDARFeature>& scan_data_raw,
                     bool correct_position = false);

  static void DownsampleCloud(const PosCloud::Ptr in_cloud,
                              PosCloud::Ptr out_cloud, float leaf_size) {
    pcl::VoxelGrid<PosPoint> sor;
    sor.setInputCloud(in_cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*out_cloud);
  }

  void UndistortScanInMap(std::shared_ptr<Trajectory> trajectory,
                          const std::vector<LiDARFeature>& scan_data_raw,
                          bool correct_position = true);

  void UndistortScanInMap(const Eigen::aligned_vector<OdomData>& odom_data);
  const std::map<double, LiDARFeature>& get_scan_data() const {
    return scan_data_;
  }

  const std::map<double, LiDARFeature>& get_scan_data_in_map() const {
    return scan_data_in_map_;
  }

  const PosCloud::Ptr& get_map_cloud() const {
    //      map_cloud_->is_dense = false;
    return map_cloud_;
  }

 private:
  void Undistort(std::shared_ptr<Trajectory> trajectory,
                 const Eigen::Quaterniond& q_G_to_target,
                 const Eigen::Vector3d& p_target_in_G,
                 const PosCloud::Ptr& scan_raw, PosCloud::Ptr& scan_in_target,
                 bool correct_position = false,
                 const PosCloud::Ptr& scan_raw_measure = nullptr) const;

  std::map<double, LiDARFeature> scan_data_;
  std::map<double, LiDARFeature> scan_data_in_map_;

  PosCloud::Ptr map_cloud_;
};

}  // namespace liso

#endif
