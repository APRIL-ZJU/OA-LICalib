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

#ifndef LIDAR_FEATURE_H
#define LIDAR_FEATURE_H

#include <ceres/ceres.h>
#include <sensor_data/cloud_type.h>
#include <Eigen/Eigen>

namespace liso {

enum LidarModelType {
  VLP_16_packet = 0,
  VLP_16_SIMU,
  VLP_16_points,
  VLP_32E_points,
  Ouster,
  Ouster_16_points,
  Ouster_32_points,
  Ouster_64_points,
  Ouster_128_points,
  LIVOX_HORIZON,
  RS_16,
  RS_M1,
};

struct LiDARFeature {
  LiDARFeature()
      : timestamp(0),
        time_max(0),
        full_features(new PosCloud),
        raw_data(new PosCloud) {}

  LiDARFeature(const LiDARFeature& fea) { *this = fea; }

  void set(LiDARFeature* lf1, LiDARFeature* lf2) {
    lf1->timestamp = lf2->timestamp;
    lf1->time_max = lf2->time_max;
    *lf1->full_features = *lf2->full_features;
    *lf1->raw_data = *lf2->raw_data;
  }

  void Clear() {
    timestamp = 0;
    time_max = 0;
    full_features->clear();
    raw_data->clear();
  }

  double timestamp;
  double time_max;  // [timestamp, max_time] full_features
  PosCloud::Ptr full_features;
  PosCloud::Ptr raw_data;
};

enum GeometryType { Line = 0, Plane };

struct PointCorrespondence {
  double t_point;
  double t_map;
  Eigen::Vector3d point;
  Eigen::Vector3d point_raw;

  GeometryType geo_type;
  Eigen::Vector4d geo_plane;

  Eigen::Vector3d geo_normal;
  Eigen::Vector3d geo_point;
};

struct LiDARIntrinsic {
 private:
  /// range: scale, offset
  /// position offset: vert_offset, horiz_offset
  /// rotation offset: vert_rad, delta_horiz_rad
  std::vector<std::vector<double>> laser_param_vec;  // 16*6

  // -15, 1, -13, 3,  -11, 5,  -9, 7, -7,  9, -5,  11, -3, 13, -1, 15
  // [case 0] time 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
  // [case 1] decresing angle 15 13 11 9 7 5 3 1 14 12 10 8 6 4 2 0
  // [case 2] incresing angle  0 2 4 6 8 10 12 14 1 3 5 7 9 11 13 15
  float vert_correction[16] = {-15, 1, -13, 3,  -11, 5,  -9, 7,
                               -7,  9, -5,  11, -3,  13, -1, 15};

  //  for different dataset
  int ring_case;

  /// Usage of different cases: vert_correction[ring_map[ring_case][dsr]]
  int ring_map[3][16] = {
      // [case 0]
      {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
      // [case 1]
      {15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0},
      // [case 2]
      {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15}};

 public:
  LiDARIntrinsic(int _ring_case = 0) : ring_case(_ring_case) {
    for (size_t dsr = 0; dsr < 16; dsr++) {
      std::vector<double> laser_param(6, 0);
      double vert_rad = vert_correction[dsr] * M_PI / 180.;
      laser_param.at(0) = 1.0;
      laser_param.at(4) = vert_rad;
      // laser_param_vec in [case 0]
      laser_param_vec.push_back(laser_param);
    }
  }

  void SetRingCase(int _ring_case) {
    assert(_ring_case >= 0 && _ring_case <= 2 && "ring_case not in range");
    ring_case = _ring_case;

    std::cout << "\033[33m"; /* Yellow */
    if (ring_case == 0)
      std::cout << "\t-lidar [ring] field in the order of firing time";
    else if (ring_case == 1)
      std::cout << "\t-lidar [ring] in the order of decreasing vertical angle";
    else if (ring_case == 2)
      std::cout << "\t-lidar [ring] in the order of incremental vertical angle";
    std::cout << "\033[0m" << std::endl;
  }

  void GetCalibrated(int laser_id, const Eigen::Vector3d& point_raw,
                     Eigen::Vector3d& point_xyz_out) {
    int ring = ring_map[ring_case][laser_id];
    GetCalibrated<double>(laser_param_vec[ring], point_raw, point_xyz_out);
  }

  template <typename T>
  static void GetCalibrated(const std::vector<T>& laser_param,
                            const Eigen::Matrix<T, 3, 1>& point_raw,
                            Eigen::Matrix<T, 3, 1>& point_xyz_out) {
    T dist_scale = laser_param[0];
    T dist_offset = laser_param[1];
    T vert_offset = laser_param[2];
    T horiz_offset = laser_param[3];
    T vert_rad = laser_param[4];
    T delta_horiz_rad = laser_param[5];

    // laser_id, angle, range
    T rot_measure = point_raw[1];
    T distance = point_raw[2];

    T rho = dist_scale * distance + dist_offset;
    T xy_dist = rho * ceres::cos(vert_rad);

    // velodyne: x-back y-right z-up
    Eigen::Matrix<T, 3, 1> point;
    point[0] = xy_dist * ceres::sin(rot_measure - delta_horiz_rad);
    point[1] = xy_dist * ceres::cos(rot_measure - delta_horiz_rad);
    point[2] = rho * ceres::sin(vert_rad);

    Eigen::Matrix<T, 3, 1> point_offset;
    point_offset[0] = horiz_offset * ceres::cos(rot_measure - delta_horiz_rad);
    point_offset[1] = horiz_offset * ceres::sin(rot_measure - delta_horiz_rad);
    point_offset[2] = vert_offset;

    point += point_offset;

    /** Use standard ROS coordinate system (right-hand rule) */
    point_xyz_out[0] = point[1];
    point_xyz_out[1] = -point[0];
    point_xyz_out[2] = point[2];
  }

  double* GetLaserParam(int laser_id, int idx) {
    assert(laser_id >= 0 && laser_id < 16 && idx < 6 && idx > 0 &&
           "[GetLaserParam] index not in range");
    return &laser_param_vec.at(ring_map[ring_case][laser_id]).at(idx);
  }

  const std::vector<std::vector<double>>& GetLaserParamVec() const {
    return laser_param_vec;
  }

  void ShowLaserParam() const {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "dist_scale, dist_offset_mm, vert_offset_mm, horiz_offset_mm, "
              << "vert_degree, delta_horiz_degree\n";
    const int(&order)[16] = ring_map[1];  // visualize in case1
    for (size_t dsr = 0; dsr < 16; dsr++) {
      std::vector<double> v = laser_param_vec.at(order[dsr]);
      std::cout << std::setw(15) << v[0] << ", "               //
                << std::setw(15) << v[1] * 1000 << ", "        //
                << std::setw(15) << v[2] * 1000 << ", "        //
                << std::setw(15) << v[3] * 1000 << ", "        //
                << std::setw(15) << v[4] * 180 / M_PI << ", "  //
                << std::setw(15) << v[5] * 180 / M_PI << std::endl;
    }
    std::cout.unsetf(std::ios::fixed);
  }
};

}  // namespace liso

#endif
