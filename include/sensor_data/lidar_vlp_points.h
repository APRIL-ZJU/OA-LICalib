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

static double rad2deg = 180.0 / M_PI;

enum VelodyneType {
  VLP16 = 0,
  VLP32E = 1,  //
};

class VelodynePoints {
 public:
  typedef std::shared_ptr<VelodynePoints> Ptr;

  VelodynePoints(VelodyneType vlp_type)
      : first_msg_(true),
        has_time_field_(false),
        has_ring_field_(false),
        one_scan_angle_(360.) {
    if (VLP16 == vlp_type) {
      num_lasers_ = 16;
      num_firing_ = 1824;  // 76*24
    } else if (VLP32E == vlp_type) {
      num_lasers_ = 32;
      num_firing_ = 2170;  // 180*12
    }
    horizon_resolution_ = one_scan_angle_ / (double)num_firing_;

    if (VLP32E == vlp_type) {
      // laserID(dsr channel)
      // channel 0,1,...,31
      laserID_mapping[31] = 0;
      laserID_mapping[29] = 1;
      laserID_mapping[27] = 2;
      laserID_mapping[25] = 3;
      laserID_mapping[23] = 4;
      laserID_mapping[21] = 5;
      laserID_mapping[19] = 6;
      laserID_mapping[17] = 7;
      laserID_mapping[15] = 8;
      laserID_mapping[13] = 9;
      laserID_mapping[11] = 10;
      laserID_mapping[9] = 11;
      laserID_mapping[7] = 12;
      laserID_mapping[5] = 13;
      laserID_mapping[3] = 14;
      laserID_mapping[1] = 15;

      laserID_mapping[30] = 16;
      laserID_mapping[28] = 17;
      laserID_mapping[26] = 18;
      laserID_mapping[24] = 19;
      laserID_mapping[22] = 20;
      laserID_mapping[20] = 21;
      laserID_mapping[18] = 22;
      laserID_mapping[16] = 23;
      laserID_mapping[14] = 24;
      laserID_mapping[12] = 25;
      laserID_mapping[10] = 26;
      laserID_mapping[8] = 27;
      laserID_mapping[6] = 28;
      laserID_mapping[4] = 29;
      laserID_mapping[2] = 30;
      laserID_mapping[0] = 31;
    }
  }

  bool check_cloud_field(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                         std::string descri) const {
    bool has_the_field = false;
    for (size_t i = 0; i < cloud_msg->fields.size(); ++i) {
      if (cloud_msg->fields[i].name == descri) {
        has_the_field = true;
        break;
      }
    }

    if (!has_the_field) {
      std::cout << "\n\t PointCloud2 not has channel [" << descri
                << "].\n\t please configure your point cloud data!";
      ROS_WARN("PointCloud2 not has channel [%s]", descri.c_str());
    }
    return has_the_field;
  }

  double ClockwiseAngle(double bef_angle, double after_angle) const {
    // atan2(py, px)
    // 1quadrant-1quadrant = 45 - 30
    // 1quadrant-4quadrant = 45 - (-30) = 75
    // 1quadrant-3quadrant = 45 - (-120) = 165
    // 1quadrant-2quadrant = 45 - 120 = -75 + 360 = 285
    double d_angle = bef_angle - after_angle;
    if (d_angle < 0) d_angle += 360;

    return d_angle;
  }

  double RotationTravelledClockwise(double now_angle,
                                    bool reset_cnt = false) const {
    static double start_angle_degree = 0;
    static bool half_rot_pointer = false;
    static double rot_circle_cnt = 0;

    if (reset_cnt) {
      start_angle_degree = now_angle;
      half_rot_pointer = false;
      rot_circle_cnt = 0;

      return 0;
    } else {
      double d_angle = ClockwiseAngle(start_angle_degree, now_angle);
      // half a circle
      if (d_angle > 100 && d_angle < 270) {
        half_rot_pointer = true;
      }
      // a circle
      if (half_rot_pointer && d_angle < 80) {
        half_rot_pointer = false;
        rot_circle_cnt += 360;
      }

      return rot_circle_cnt + d_angle;  // rot_travelled
    }
  }

  bool InitScanParam(const sensor_msgs::PointCloud2::ConstPtr &lidarMsg) {
    has_time_field_ = check_cloud_field(lidarMsg, "time");
    has_ring_field_ = check_cloud_field(lidarMsg, "ring");

    // if (!has_ring_field_) return false;

    if (!has_time_field_) {
      ROS_WARN(
          "Input PointCloud2 not has channel [time]. Calculate timestamp "
          "of each point assuming constant rotation speed");
    }

    VPointCloud cloud;
    pcl::fromROSMsg(*lidarMsg, cloud);

    // 起始位置标记
    bool is_first_horizon_angle = true;

    double rotation_travelled = 0;
    for (auto const &point_in : cloud) {
      if (pcl_isnan(point_in.x) || pcl_isnan(point_in.y) ||
          pcl_isnan(point_in.z))
        continue;
      // std::cout << point_in.x << ", " << point_in.y << "\n";

      double horizon_angle = atan2(point_in.y, point_in.x) * rad2deg;
      if (is_first_horizon_angle) {
        is_first_horizon_angle = false;
        RotationTravelledClockwise(horizon_angle, true);
      }
      rotation_travelled = RotationTravelledClockwise(horizon_angle);
    }

    one_scan_angle_ = round(rotation_travelled / 360.) * 363.;

    horizon_resolution_ = one_scan_angle_ / (double)num_firing_;

    std::cout << "\t one_scan_angle: " << one_scan_angle_ << std::endl;
    std::cout << "\t horizon_resolution: " << horizon_resolution_ << std::endl;

    return true;
  }

  void get_organized_and_raw_cloud(
      const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
      LiDARFeature &output) {
    if (first_msg_) {
      bool ret = InitScanParam(lidarMsg);
      // if (!ret) return;

      first_msg_ = false;
    }

    static double lidar_fov_down = -15.0;
    static double lidar_fov_resolution = 2.0;

    RTPointCloud pc_in;
    pcl::fromROSMsg(*lidarMsg, pc_in);

    double timebase = lidarMsg->header.stamp.toSec();
    output.timestamp = timebase;

    /// point cloud
    output.full_features->clear();
    output.full_features->height = num_lasers_;
    output.full_features->width = num_firing_;
    output.full_features->is_dense = false;
    output.full_features->resize(output.full_features->height *
                                 output.full_features->width);

    /// raw_data
    output.raw_data->height = num_lasers_;
    output.raw_data->width = num_firing_;
    output.raw_data->is_dense = false;
    output.raw_data->resize(output.raw_data->height * output.raw_data->width);

    PosPoint NanPoint;
    NanPoint.x = NAN;
    NanPoint.y = NAN;
    NanPoint.z = NAN;
    NanPoint.timestamp = timebase;

    int num_points = num_lasers_ * num_firing_;
    for (int k = 0; k < num_points; k++) {
      output.full_features->points[k] = NanPoint;
      output.raw_data->points[k] = NanPoint;
    }

    int last_firing = 0;
    int firing_order_positive = 0, firing_order_positive_cnt = 0;
    int firing_order_reverse = 0, firing_order_reverse_cnt = 0;

    bool is_first_horizon_angle = true;
    for (int i = 0; i < pc_in.size(); ++i) {
      const RTPoint &point_in = pc_in.points[i];

      if (pcl_isnan(point_in.x) || pcl_isnan(point_in.y) ||
          pcl_isnan(point_in.z))
        continue;

      double horizon_angle = atan2(point_in.y, point_in.x) * rad2deg;
      if (is_first_horizon_angle) {
        is_first_horizon_angle = false;
        RotationTravelledClockwise(horizon_angle, true);
      }
      double rotation_travelled = RotationTravelledClockwise(horizon_angle);

      int firing = round(rotation_travelled / horizon_resolution_);

      if (i > 0) {
        int df = firing - last_firing;
        if (df >= 0) {
          firing_order_positive += df;
          firing_order_positive_cnt++;
        } else {
          firing_order_reverse += df;
          firing_order_reverse_cnt++;
        }
      }
      last_firing = firing;
      // std::cout << "[" << horizon_angle << ", " << firing << ", "
      //           << point_in.ring << "]; z =" << point_in.z << "\n";

      if (firing < 0 || firing >= num_firing_) continue;

      double dt = 0;
      if (has_time_field_) {
        dt = point_in.time;
      } else {
        dt = 0.1 * rotation_travelled / one_scan_angle_;
      }

      double depth = sqrt(point_in.x * point_in.x + point_in.y * point_in.y +
                          point_in.z * point_in.z);

      int ring = 0;
      if (has_ring_field_) {
        ring = point_in.ring;
      } else {
        double pitch = asin(point_in.z / depth) * rad2deg;
        ring = std::round((pitch - lidar_fov_down) / lidar_fov_resolution);
        if (ring < 0 || ring >= 16) continue;
      }

      PosPoint point_out;
      point_out.x = point_in.x;
      point_out.y = point_in.y;
      point_out.z = point_in.z;
      point_out.timestamp = timebase + dt;

      PosPoint point_raw;
      point_raw.timestamp = dt;
      point_raw.x = ring;
      point_raw.y = horizon_angle / rad2deg;
      point_raw.z = depth;

      if (point_raw.z > 40) continue;

      output.full_features->at(firing, ring) = point_out;
      output.raw_data->at(firing, ring) = point_raw;
    }

    if (firing_order_reverse_cnt > 10) {
      std::cout << "firing order counter [positive/reverse] = ["
                << "/" << firing_order_positive_cnt << "/"
                << firing_order_reverse_cnt << "]\n";

      ROS_WARN("check out the caculation of horizon_angle.");
    }
    // pcl::io::savePCDFileBinaryCompressed(
    //     "/home/ha/ros_ws/catkin_liso/pc_out.pcd", *output.full_features);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "/home/ha/ros_ws/catkin_liso/pc_in.pcd", pc_in);
  }

 private:
  int num_firing_;
  int num_lasers_;

  bool first_msg_;
  bool has_time_field_;
  bool has_ring_field_;

  double horizon_resolution_;
  double one_scan_angle_;

  int laserID_mapping[32];
};
}  // namespace liso
