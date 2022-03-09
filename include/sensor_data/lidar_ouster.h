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

#include <angles/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_data/lidar_feature.h>
#include <sensor_msgs/PointCloud2.h>

namespace liso {

enum OusterRingNo {
  Ring128 = 128,
  Ring64 = 64,
  Ring32 = 32,
  Ring16 = 16,  //
};

class OusterLiDAR {
 public:
  typedef std::shared_ptr<OusterLiDAR> Ptr;

  OusterLiDAR(OusterRingNo ring_no)
      : ouster_ring_No_(ring_no), num_firing_(2048) {}

  void get_organized_and_raw_cloud(
      const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
      LiDARFeature &output) {
    OusterPointCloud pc_in;
    pcl::fromROSMsg(*lidarMsg, pc_in);

    int ring_number = int(ouster_ring_No_);
    int ring_step = pc_in.height / ring_number;

    assert(ring_step >= 1 && "OusterRingNo too large");

    double timebase = lidarMsg->header.stamp.toSec();
    output.timestamp = timebase;

    /// point cloud
    output.full_features->clear();
    output.full_features->height = ring_number;
    output.full_features->width = num_firing_;
    output.full_features->is_dense = false;
    output.full_features->resize(output.full_features->height *
                                 output.full_features->width);

    /// raw_data
    output.raw_data->height = ring_number;
    output.raw_data->width = num_firing_;
    output.raw_data->is_dense = false;
    output.raw_data->resize(output.raw_data->height * output.raw_data->width);

    PosPoint NanPoint;
    NanPoint.x = NAN;
    NanPoint.y = NAN;
    NanPoint.z = NAN;
    NanPoint.timestamp = timebase;

    int num_points = ring_number * num_firing_;
    for (int k = 0; k < num_points; k++) {
      output.full_features->points[k] = NanPoint;
      output.raw_data->points[k] = NanPoint;
    }

    for (int h = 0; h < ring_number; h++) {
      int h_in_selected = h * ring_step;
      for (int w = 0; w < num_firing_; w++) {
        const auto &src = pc_in.at(w, h_in_selected);

        // std::cout << w << "," << h << "," << src.x << "," << src.y
        //           << "," << src.z << "," << src.t << "\n";

        double depth = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
        if (depth > 60) continue;

        PosPoint dst_point;
        dst_point.x = src.x;
        dst_point.y = src.y;
        dst_point.z = src.z;
        dst_point.timestamp = timebase + src.t * 1e-9f;

        PosPoint point_raw;
        // t_offset wrt. first point
        point_raw.timestamp = src.t * 1e-9f;
        // laser id
        point_raw.x = src.ring;
        // angle rad
        point_raw.y = src.t;
        // range m
        point_raw.z = depth;

        output.full_features->at(w, h) = dst_point;
        output.raw_data->at(w, h) = point_raw;
      }
    }
    // pcl::io::savePCDFileBinaryCompressed(
    //     "/home/ha/ros_ws/catkin_liso/ouster_cloud_in.pcd", pc_in);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "/home/ha/ros_ws/catkin_liso/ouster_cloud_out.pcd",
    //     *output.full_features);
  }

 private:
  OusterRingNo ouster_ring_No_;

  int num_firing_;
};
}  // namespace liso
