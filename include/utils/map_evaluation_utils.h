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

#ifndef MAP_EVALUATION_UTILS_HPP
#define MAP_EVALUATION_UTILS_HPP

#include <omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

/// Reference
/// https://github.com/AIS-Bonn/pointcloud_evaluation_tool
/// http://www.ais.uni-bonn.de/papers/ECMR_2015_Razlaw.pdf

struct PointTypeWithEntropy {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ + padding
  float entropy;
  float planeVariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointTypeWithEntropy,
    (float, x, x)(float, y, y)(float, z, z)(float, entropy,
                                            entropy)(float, planeVariance,
                                                     planeVariance))

class MapEvaluationTool {
 public:
  MapEvaluationTool(std::string map_path, int step_size = 1,
                    double radius = 0.3, int min_neighbors = 15,
                    bool punish_solitary_points = false)
      : map_path_(map_path),
        step_size_(step_size),
        radius_(radius),
        min_neighbors_(min_neighbors),
        punish_solitary_points_(punish_solitary_points) {}

  double ComputeEntropy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covarianceMatrixNormalized = Eigen::Matrix3f::Identity();
    ;

    // estimate the XYZ centroid and the normalized covariance matrix
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid,
                                           covarianceMatrixNormalized);

    // compute the determinant and return the entropy
    double determinant = static_cast<double>(
        ((2 * M_PI * M_E) * covarianceMatrixNormalized).determinant());

    return 0.5f * log(determinant);
  }

  double ComputePlaneVariance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    double meanDistTopQuarter = 0;
    std::vector<double> sortedDistances;

    // fit plane using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 3) {
      PCL_ERROR(
          "Could not estimate a planar model for the given subset of points.");
      meanDistTopQuarter = std::numeric_limits<double>::infinity();
    } else {
      // compute the distances of the points to the plane
      for (size_t i = 0; i < cloud->points.size(); ++i) {
        double distancePointToPlane =
            (cloud->points[i].x * coefficients->values[0]) +
            (cloud->points[i].y * coefficients->values[1]) +
            (cloud->points[i].z * coefficients->values[2]) +
            coefficients->values[3];
        sortedDistances.push_back(fabs(distancePointToPlane));
      }
      // sort distances
      std::sort(sortedDistances.begin(), sortedDistances.end());

      // compute mean of quartile that contains the largest distances
      int quarterOfArray = sortedDistances.size() / 4;
      for (size_t i = quarterOfArray * 3; i < sortedDistances.size(); i++) {
        meanDistTopQuarter += sortedDistances[i];
      }
      meanDistTopQuarter /= static_cast<double>(quarterOfArray);
    }

    return meanDistTopQuarter;
  }

  bool Process() {
    double entropy_sum = 0;
    double plane_variance_sum = 0;
    double lonely_points = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointTypeWithEntropy>::Ptr output_cloud(
        new pcl::PointCloud<PointTypeWithEntropy>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *input_cloud) == -1) {
      return false;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);

#pragma omp parallel reduction (+:entropy_sum, plane_variance_sum, lonely_points)
    {
#pragma omp for schedule(dynamic)
      for (size_t i = 0; i < input_cloud->points.size(); i += step_size_) {
        // print status
        if (i % (input_cloud->points.size() / 20) == 0) {
          int percent = i * 100 / input_cloud->points.size();
          std::cout << percent << " %" << std::endl;
        }

        // search for neighbors in radius
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        int numberOfNeighbors = kdtree.radiusSearch(
            input_cloud->points[i], radius_, pointIdxRadiusSearch,
            pointRadiusSquaredDistance);

        // compute values if enough neighbors found
        double localEntropy = 0;
        double localPlaneVariance = 0;
        if (numberOfNeighbors > min_neighbors_ || !punish_solitary_points_) {
          // save neighbors in localCloud
          pcl::PointCloud<pcl::PointXYZ>::Ptr localCloud(
              new pcl::PointCloud<pcl::PointXYZ>);

          for (size_t iz = 0; iz < pointIdxRadiusSearch.size(); ++iz) {
            localCloud->points.push_back(
                input_cloud->points[pointIdxRadiusSearch[iz]]);
          }

          // compute entropy and plane variance
          localEntropy = ComputeEntropy(localCloud);
          localPlaneVariance = ComputePlaneVariance(localCloud);
        } else {
          localEntropy = std::numeric_limits<double>::infinity();
          localPlaneVariance = std::numeric_limits<double>::infinity();
          lonely_points++;
        }

        // save values in new point
        PointTypeWithEntropy p;
        p.x = input_cloud->points[i].x;
        p.y = input_cloud->points[i].y;
        p.z = input_cloud->points[i].z;

        if (pcl_isfinite(localPlaneVariance)) {
          plane_variance_sum += localPlaneVariance;
          p.planeVariance = static_cast<float>(localPlaneVariance);
        } else {
          // handle cases where no value could be computed
          if (!punish_solitary_points_) {
            p.planeVariance = 0;
          } else {
            plane_variance_sum += radius_;
            p.planeVariance = static_cast<float>(radius_);
          }
        }
        if (pcl_isfinite(localEntropy)) {
          entropy_sum += localEntropy;
          p.entropy = static_cast<float>(localEntropy);
        } else {
          // handle cases where no value could be computed
          p.entropy = 0;
        }

// add new point to output cloud
#pragma omp critical
        { output_cloud->push_back(p); }
      }
    }

    double mean_map_entropy =
        entropy_sum /
        (static_cast<double>(input_cloud->points.size() / step_size_));
    double mean_plane_variance =
        plane_variance_sum /
        (static_cast<double>(input_cloud->points.size() / step_size_));

    std::cout << "--- " << std::endl;
    std::cout << "Mean Map Entropy is :  " << mean_map_entropy << std::endl;
    std::cout << "Mean Plane Variance is : " << mean_plane_variance
              << std::endl;

    std::string MME_file = map_path_.substr(0, map_path_.find_last_of("/"));
    MME_file = MME_file.substr(0, MME_file.find_last_of("/")) + "/map_MME.txt";

    std::ofstream outfile;
    outfile.open(MME_file, std::ios::app);
    outfile << map_path_ << "," << mean_map_entropy << ","
            << mean_plane_variance << std::endl;
    outfile.close();

    std::string save_entropy_cloud = map_path_;
    save_entropy_cloud.replace(save_entropy_cloud.find_last_of("."), 1,
                               "_entrop.");
    if (output_cloud->size() > 0) {
      pcl::io::savePCDFileBinaryCompressed(save_entropy_cloud, *output_cloud);
    } else {
      PCL_ERROR("Empty cloud. Saving error.\n");
    }

    return true;
  }

 private:
  std::string map_path_;

  int step_size_;

  double radius_;

  int min_neighbors_;

  bool punish_solitary_points_;
};

#endif
