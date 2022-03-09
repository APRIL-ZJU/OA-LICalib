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

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utils/eigen_utils.hpp>

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

/// reference
/// https://github.com/ros-drivers/velodyne/blob/master/velodyne_pcl/include/velodyne_pcl/point_types.h
namespace velodyne_pcl {
struct PointXYZIRT {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  float time;                      ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZT {
  PCL_ADD_POINT4D;                 /// quad-word XYZ
  double timestamp;                /// laser timestamp
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

}  // namespace velodyne_pcl

// Ouster lidar
struct PointXYZIR8Y {
  PCL_ADD_POINT4D;  // quad-word XYZ
  float intensity;  ///< laser intensity reading
  uint8_t ring;     ///< laser ring number
  uint32_t t;
  float range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZIRT,     //
                                  (float, x, x)                  //
                                  (float, y, y)                  //
                                  (float, z, z)                  //
                                  (float, intensity, intensity)  //
                                  (uint16_t, ring, ring)         //
                                  (float, time, time)            //
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZT,        //
                                  (float, x, x)                   //
                                  (float, y, y)(float, z, z)      //
                                  (double, timestamp, timestamp)  //
)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR8Y,                  //
                                  (float, x, x)                  //
                                  (float, y, y)                  //
                                  (float, z, z)                  //
                                  (float, intensity, intensity)  //
                                  (uint8_t, ring, ring)          //
                                  (uint32_t, t, t)               //
)

typedef velodyne_pcl::PointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;

typedef velodyne_pcl::PointXYZT PosPoint;
typedef pcl::PointCloud<PosPoint> PosCloud;

typedef PointXYZIR8Y OusterPoint;
typedef pcl::PointCloud<OusterPoint> OusterPointCloud;

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
