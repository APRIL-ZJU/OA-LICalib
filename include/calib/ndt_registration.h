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

#ifndef NDT_REGISTRATION_HPP
#define NDT_REGISTRATION_HPP
#define PCL_NO_PRECOMPILE

#include <sensor_data/cloud_type.h>
#include <sensor_data/lidar_feature.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <pclomp/ndt_omp.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

namespace liso {

class NDTRegistration {
 public:
  typedef std::shared_ptr<NDTRegistration> Ptr;

  NDTRegistration(const YAML::Node& node) {
    ndt_resolution_ = node["ndt_resolution"].as<double>();
    ndt_omp_ = pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr(
        new pclomp::NormalDistributionsTransform<PosPoint, PosPoint>());
    ndt_omp_->setResolution(ndt_resolution_);
    ndt_omp_->setNumThreads(6);
    ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt_omp_->setTransformationEpsilon(1e-3);
    ndt_omp_->setStepSize(0.01);
    ndt_omp_->setMaximumIterations(50);
  }

  NDTRegistration(double ndt_resolution) {
    ndt_resolution_ = ndt_resolution;
    //    ndt_omp_ = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
    //            (new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
    ndt_omp_ = pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr(
        new pclomp::NormalDistributionsTransform<PosPoint, PosPoint>());
    ndt_omp_->setResolution(ndt_resolution_);
    ndt_omp_->setNumThreads(6);
    ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt_omp_->setTransformationEpsilon(1e-3);
    ndt_omp_->setStepSize(0.01);
    ndt_omp_->setMaximumIterations(50);
  }

  bool SetInputTarget(const LiDARFeature& feature_map) {
    ndt_omp_->setInputTarget(feature_map.full_features);
    return true;
  }

  bool ScanMatch(const LiDARFeature& feature_cur,
                 const Eigen::Matrix4d& predict_pose,
                 LiDARFeature& result_feature, Eigen::Matrix4d& result_pose) {
    PosCloud::Ptr filter_cloud(new PosCloud());
    DownsampleCloud(feature_cur.full_features, filter_cloud, 0.5);
    ndt_omp_->setInputSource(filter_cloud);
    Eigen::Matrix4f init_pose = predict_pose.cast<float>();
    PosCloud output_cloud;  // unused
    ndt_omp_->align(output_cloud, init_pose);
    if (!ndt_omp_->hasConverged()) {
      std::cout << "NDT does not converge!!!" << std::endl;
      return false;
    }
    result_pose = ndt_omp_->getFinalTransformation().cast<double>();
    // PosCloud::Ptr transform_cloud(new PosCloud);
    // pcl::transformPointCloud(*result_feature.full_features, *transform_cloud,
    //                          result_pose);
    // result_feature.timestamp = feature_cur.timestamp;
    // result_feature.full_features = transform_cloud;
    return true;
  }

  const pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr&
  get_ndt_ptr() const {
    return ndt_omp_;
  }

 private:
  void DownsampleCloud(PosCloud::Ptr in_cloud, PosCloud::Ptr out_cloud,
                       float leaf_size) {
    pcl::VoxelGrid<PosPoint> sor;
    sor.setInputCloud(in_cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*out_cloud);
  }

  VPointCloud::Ptr TransformToVPointCloud(PosCloud::Ptr cloud_in) {
    VPointCloud::Ptr result(new VPointCloud);
    for (size_t i = 0; i < cloud_in->size(); i++) {
      VPoint p;
      p.x = cloud_in->points[i].x;
      p.y = cloud_in->points[i].y;
      p.z = cloud_in->points[i].z;
      result->push_back(p);
    }
    return result;
  }

  pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr ndt_omp_;

  double ndt_resolution_;
};

}  // namespace liso

#endif
