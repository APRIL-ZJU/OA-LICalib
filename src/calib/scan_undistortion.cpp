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

#include <calib/scan_undistortion.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>

namespace liso {

/// [improve NDT accuracy] remove rotaional motion distortion
void ScanUndistortion::UndistortScan(
    std::shared_ptr<Trajectory> trajectory,
    const std::vector<LiDARFeature>& scan_data_raw, bool correct_position) {
  scan_data_.clear();
  for (const LiDARFeature& scan_raw : scan_data_raw) {
    double scan_timestamp = scan_raw.timestamp;
    SE3d pose = trajectory->GetLidarPose(scan_timestamp);
    Eigen::Quaterniond q_L0_to_G = pose.unit_quaternion();
    Eigen::Vector3d p_L0_in_G = pose.translation();

    PosCloud::Ptr scan_in_target(new PosCloud);
    Undistort(trajectory, q_L0_to_G.conjugate(), p_L0_in_G,
              scan_raw.full_features, scan_in_target, correct_position);

    LiDARFeature undistort_feature;
    undistort_feature.timestamp = scan_raw.timestamp;
    undistort_feature.full_features = scan_in_target;

    scan_data_.insert({scan_raw.timestamp, undistort_feature});
  }
}

/// transfrom scna data to map frame and build map based on trajectory
void ScanUndistortion::UndistortScanInMap(
    std::shared_ptr<Trajectory> trajectory,
    const std::vector<LiDARFeature>& scan_data_raw, bool correct_position) {
  scan_data_in_map_.clear();
  map_cloud_ = PosCloud::Ptr(new PosCloud);

  //      double map_start_time = trajectory->minTime();
  //      SE3d map_start_pose = trajectory->GetLidarPose(map_start_time);
  //      Eigen::Quaterniond q_L0_to_G = map_start_pose.unit_quaternion();
  //      Eigen::Vector3d p_L0_in_G = map_start_pose.translation();
  Eigen::Quaterniond q_L0_to_G = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_L0_in_G = Eigen::Vector3d(0, 0, 0);

  bool apply_lidar_intrinstic =
      trajectory->GetCalibParam()->calib_option.apply_lidar_intrinstic_to_scan;

  double filter_voxel_size =
      trajectory->GetCalibParam()->lo_param.ndt_key_frame_downsample;

  int cnt = 0;
  for (const LiDARFeature& scan_raw : scan_data_raw) {
    // if (cnt++ % 3 != 0) continue;
    double scan_timestamp = scan_raw.timestamp;
    if (!trajectory->GetLiDARTrajQuality(scan_timestamp)) continue;
    PosCloud::Ptr scan_in_target(new PosCloud);
    if (apply_lidar_intrinstic) {
      Undistort(trajectory, q_L0_to_G.conjugate(), p_L0_in_G,
                scan_raw.full_features, scan_in_target, correct_position,
                scan_raw.raw_data);
    } else {
      Undistort(trajectory, q_L0_to_G.conjugate(), p_L0_in_G,
                scan_raw.full_features, scan_in_target, correct_position,
                nullptr);
    }

    LiDARFeature undistort_feature;
    undistort_feature.timestamp = scan_raw.timestamp;
    undistort_feature.full_features = scan_in_target;

    scan_data_in_map_.insert({scan_raw.timestamp, undistort_feature});

    ///  对 scan 降采样
    PosCloud::Ptr scan_in_target_ds(new PosCloud);
    scan_in_target_ds->points.reserve(scan_in_target->size());
#if false
    DownsampleCloud(scan_in_target, scan_in_target_ds, filter_voxel_size);
#else
    for (int k = 0; k < scan_in_target->size(); k += 3) {
      const auto& p_in = scan_in_target->points[k];
      if (pcl_isnan(p_in.x)) continue;

      PosPoint p;
      p.x = p_in.x;
      p.y = p_in.y;
      p.z = p_in.z;
      p.timestamp = p_in.timestamp;
      scan_in_target_ds->push_back(p);
    }
#endif
    *map_cloud_ += *scan_in_target_ds;
  }
}

//  transfrom scna data to map frame based on the lidar odometry
void ScanUndistortion::UndistortScanInMap(
    const Eigen::aligned_vector<OdomData>& odom_data) {
  scan_data_in_map_.clear();
  map_cloud_ = PosCloud::Ptr(new PosCloud);

  for (auto const& odom : odom_data) {
    auto iter = scan_data_.find(odom.timestamp);
    if (iter == scan_data_.end()) {
      continue;
    }
    PosCloud::Ptr scan_inMap = PosCloud::Ptr(new PosCloud);
    pcl::transformPointCloud(*(iter->second.full_features), *scan_inMap,
                             odom.pose);

    LiDARFeature undistort_feature;
    undistort_feature.timestamp = odom.timestamp;
    undistort_feature.full_features = scan_inMap;

    scan_data_in_map_.insert({odom.timestamp, undistort_feature});
    *map_cloud_ += *scan_inMap;
  }
}

void ScanUndistortion::Undistort(std::shared_ptr<Trajectory> trajectory,
                                 const Eigen::Quaterniond& q_G_to_target,
                                 const Eigen::Vector3d& p_target_in_G,
                                 const PosCloud::Ptr& scan_raw,
                                 PosCloud::Ptr& scan_in_target,
                                 bool correct_position,
                                 const PosCloud::Ptr& scan_raw_measure) const {
  scan_in_target->header = scan_raw->header;
  scan_in_target->height = scan_raw->height;
  scan_in_target->width = scan_raw->width;
  scan_in_target->resize(scan_raw->height * scan_raw->width);
  scan_in_target->is_dense = false;

  bool apply_lidar_intrinsic = false;
  if (trajectory->GetCalibParam()->calib_option.opt_lidar_intrinsic &&
      scan_raw_measure) {
    apply_lidar_intrinsic = true;
  }
  const auto lidar_intrinsic = &trajectory->GetCalibParam()->lidar_intrinsic;

  PosPoint NanPoint;
  NanPoint.x = NAN;
  NanPoint.y = NAN;
  NanPoint.z = NAN;
  for (int h = 0; h < scan_raw->height; h++) {
    for (int w = 0; w < scan_raw->width; w++) {
      PosPoint vpoint;
      if (pcl_isnan(scan_raw->at(w, h).x)) {
        vpoint = NanPoint;
        scan_in_target->at(w, h) = vpoint;
        continue;
      }
      double point_timestamp = scan_raw->at(w, h).timestamp;

      SE3d point_pose;
      if (!trajectory->GetLidarPose(point_timestamp, point_pose)) continue;
      Eigen::Quaterniond q_Lk_to_G = point_pose.unit_quaternion();
      Eigen::Vector3d p_Lk_in_G = point_pose.translation();

      Eigen::Quaterniond q_LktoL0 = q_G_to_target * q_Lk_to_G;
      Eigen::Vector3d p_Lk;
      if (!apply_lidar_intrinsic) {
        p_Lk = Eigen::Vector3d(scan_raw->at(w, h).x, scan_raw->at(w, h).y,
                               scan_raw->at(w, h).z);
      } else {
        Eigen::Vector3d point_raw(scan_raw_measure->at(w, h).x,
                                  scan_raw_measure->at(w, h).y,
                                  scan_raw_measure->at(w, h).z);
        int dsr = int(point_raw[0]);
        lidar_intrinsic->GetCalibrated(dsr, point_raw, p_Lk);
      }

      Eigen::Vector3d point_out;
      if (!correct_position) {
        point_out = q_LktoL0 * p_Lk;
      } else {
        point_out =
            q_LktoL0 * p_Lk + q_G_to_target * (p_Lk_in_G - p_target_in_G);
      }

      vpoint.x = point_out(0);
      vpoint.y = point_out(1);
      vpoint.z = point_out(2);
      vpoint.timestamp = scan_raw->at(w, h).timestamp;
      scan_in_target->at(w, h) = vpoint;
    }
  }
}

}  // namespace liso
