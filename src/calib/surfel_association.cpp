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

#include <calib/surfel_association.h>

namespace liso {

void SurfelAssociation::InitColorList() {
  color_list_.set_capacity(6);
  color_list_.push_back(0xFF0000);
  color_list_.push_back(0xFF00FF);
  color_list_.push_back(0x436EEE);
  color_list_.push_back(0xBF3EFF);
  color_list_.push_back(0xB4EEB4);
  color_list_.push_back(0xFFE7BA);
}

void SurfelAssociation::ClearSurfelMap() {
  surfel_planes_.clear();
  spoint_per_surfel_.clear();
  spoint_downsampled_.clear();
  surfels_map_.clear();
  spoints_all_.clear();
}

void SurfelAssociation::SetSurfelMap(
    const pclomp::NormalDistributionsTransform<PosPoint, PosPoint>::Ptr &ndtPtr,
    double timestamp) {
  ClearSurfelMap();
  map_timestamp_ = timestamp;
  Eigen::Vector3i counter(0, 0, 0);
  for (const auto &v : ndtPtr->getTargetCells().getLeaves()) {
    auto leaf = v.second;

    if (leaf.nr_points < 10) continue;
    int plane_type =
        CheckPlaneType(leaf.getEvals(), leaf.getEvecs(), p_lambda_);
    if (plane_type < 0) continue;

    Eigen::Vector4d surfCoeff;
    PosCloud::Ptr cloud_inliers(new PosCloud);
    if (!FitPlane(leaf.pointList_.makeShared(), surfCoeff, cloud_inliers))
      continue;

    counter(plane_type) += 1;
    SurfelPlane surfplane;
    surfplane.cloud = leaf.pointList_;
    surfplane.cloud_inlier = *cloud_inliers;
    surfplane.p4 = surfCoeff;
    surfplane.Pi = -surfCoeff(3) * surfCoeff.head<3>();

    PosPoint min, max;
    pcl::getMinMax3D<PosPoint>(surfplane.cloud, min, max);
    surfplane.boxMin = Eigen::Vector3d(min.x, min.y, min.z);
    surfplane.boxMax = Eigen::Vector3d(max.x, max.y, max.z);

    surfel_planes_.push_back(surfplane);
  }

  spoint_per_surfel_.resize(surfel_planes_.size());

  std::cout << "Plane type  :" << counter.transpose()
            << "; Plane number: " << surfel_planes_.size() << std::endl;

  surfels_map_.clear();
  {
    int idx = 0;
    for (const auto &v : surfel_planes_) {
      ColorPointCloud cloud_rgb;
      pcl::copyPointCloud(v.cloud_inlier, cloud_rgb);

      size_t colorType = (idx++) % color_list_.size();
      for (auto &p : cloud_rgb) {
        p.rgba = color_list_[colorType];
      }
      surfels_map_ += cloud_rgb;
    }
  }
}

void SurfelAssociation::GetAssociation(const PosCloud::Ptr &scan_inM,
                                       const PosCloud::Ptr &scan_raw_xyz,
                                       const PosCloud::Ptr &scan_raw_measure,
                                       size_t selected_num_per_ring) {
  const size_t width = scan_raw_xyz->width;
  const size_t height = scan_raw_xyz->height;

  int associatedFlag[width * height];
  for (unsigned int i = 0; i < width * height; i++) {
    associatedFlag[i] = -1;
  }

#pragma omp parallel for num_threads(omp_get_max_threads())
  for (int plane_id = 0; plane_id < surfel_planes_.size(); plane_id++) {
    std::vector<std::vector<int>> ring_masks;
    AssociateScanToSurfel(plane_id, scan_inM, associated_radius_, ring_masks);

    for (int h = 0; h < height; h++) {
      if (ring_masks.at(h).size() < selected_num_per_ring * 2) continue;
      int step = ring_masks.at(h).size() / (selected_num_per_ring + 1);
      step = std::max(step, 1);
      for (int selected = 0; selected < selected_num_per_ring; selected++) {
        int w = ring_masks.at(h).at(step * (selected + 1) - 1);
        associatedFlag[h * width + w] = plane_id;
      }
    }
  }

  // in chronological order
  SurfelPoint spoint;
  for (int w = 0; w < width; w++) {
    for (int h = 0; h < height; h++) {
      if (associatedFlag[h * width + w] == -1 ||
          0 == scan_raw_xyz->at(w, h).timestamp)
        continue;
      spoint.point =
          Eigen::Vector3d(scan_raw_xyz->at(w, h).x, scan_raw_xyz->at(w, h).y,
                          scan_raw_xyz->at(w, h).z);
      spoint.point_in_map = Eigen::Vector3d(
          scan_inM->at(w, h).x, scan_inM->at(w, h).y, scan_inM->at(w, h).z);
      spoint.plane_id = associatedFlag[h * width + w];
      spoint.timestamp = scan_raw_xyz->at(w, h).timestamp;
      if (scan_raw_measure) {
        spoint.point_raw = Eigen::Vector3d(scan_raw_measure->at(w, h).x,
                                           scan_raw_measure->at(w, h).y,
                                           scan_raw_measure->at(w, h).z);
      }

      spoint_per_surfel_.at(spoint.plane_id).push_back(spoint);
      spoints_all_.push_back(spoint);
    }
  }
}

void SurfelAssociation::RandomDownSample(int num_points_max) {
  for (const auto &v : spoint_per_surfel_) {
    if (v.size() < 20) continue;
    for (int i = 0; i < num_points_max; i++) {
      int random_index = rand() / (RAND_MAX)*v.size();
      spoint_downsampled_.push_back(v.at(random_index));
    }
  }
}

void SurfelAssociation::AverageDownSmaple(int num_points_max) {
  for (const auto &v : spoint_per_surfel_) {
    if (v.size() < 20) continue;
    int d_step = v.size() / num_points_max;
    int step = d_step > 1 ? d_step : 1;
    for (int i = 0; i < v.size(); i += step) {
      spoint_downsampled_.push_back(v.at(i));
    }
  }
}

void SurfelAssociation::AverageTimeDownSmaple(int step) {
  for (size_t idx = 0; idx < spoints_all_.size(); idx += step) {
    spoint_downsampled_.push_back(spoints_all_.at(idx));
  }
}

int SurfelAssociation::CheckPlaneType(const Eigen::Vector3d &eigen_value,
                                      const Eigen::Matrix3d &eigen_vector,
                                      const double &p_lambda) {
  Eigen::Vector3d sorted_vec;
  Eigen::Vector3i ind;
  Eigen::sort_vec(eigen_value, sorted_vec, ind);

  double p = 2 * (sorted_vec[1] - sorted_vec[2]) /
             (sorted_vec[2] + sorted_vec[1] + sorted_vec[0]);

  if (p < p_lambda) {
    return -1;
  }

  int min_idx = ind[2];
  Eigen::Vector3d plane_normal = eigen_vector.block<3, 1>(0, min_idx);
  plane_normal = plane_normal.array().abs();

  Eigen::sort_vec(plane_normal, sorted_vec, ind);
  return ind[2];
}

bool SurfelAssociation::FitPlane(const PosCloud::Ptr &cloud,
                                 Eigen::Vector4d &coeffs,
                                 PosCloud::Ptr cloud_inliers) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PosPoint> seg;  /// Create the segmentation object
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() < 20) {
    return false;
  }

  for (int i = 0; i < 4; i++) {
    coeffs(i) = coefficients->values[i];
  }

  pcl::copyPointCloud<PosPoint>(*cloud, *inliers, *cloud_inliers);
  return true;
}

double SurfelAssociation::Point2PlaneDistance(Eigen::Vector3d &pt,
                                              Eigen::Vector4d &plane_coeff) {
  Eigen::Vector3d normal = plane_coeff.head<3>();
  double dist = pt.dot(normal) + plane_coeff(3);
  dist = dist > 0 ? dist : -dist;

  return dist;
}

void SurfelAssociation::AssociateScanToSurfel(
    const size_t &surfel_idx, const PosCloud::Ptr &scan, const double &radius,
    std::vector<std::vector<int>> &ring_masks) const {
  Eigen::Vector3d box_min = surfel_planes_.at(surfel_idx).boxMin;
  Eigen::Vector3d box_max = surfel_planes_.at(surfel_idx).boxMax;
  Eigen::Vector4d plane_coeffs = surfel_planes_.at(surfel_idx).p4;

  for (int j = 0; j < scan->height; j++) {
    std::vector<int> mask_per_ring;
    for (int i = 0; i < scan->width; i++) {
      if (!pcl_isnan(scan->at(i, j).x) && scan->at(i, j).x > box_min[0] &&
          scan->at(i, j).x < box_max[0] && scan->at(i, j).y > box_min[1] &&
          scan->at(i, j).y < box_max[1] && scan->at(i, j).z > box_min[2] &&
          scan->at(i, j).z < box_max[2]) {
        Eigen::Vector3d point(scan->at(i, j).x, scan->at(i, j).y,
                              scan->at(i, j).z);
        if (Point2PlaneDistance(point, plane_coeffs) <= radius) {
          mask_per_ring.push_back(i);
        }
      }
    }  // end of one colmun (ring)
    ring_masks.push_back(mask_per_ring);
  }
}
}  // namespace liso
