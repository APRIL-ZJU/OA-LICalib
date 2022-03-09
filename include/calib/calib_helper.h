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

#ifndef CALIB_HELPER_H
#define CALIB_HELPER_H

#include <ros/package.h>
#include <string>

#include <calib/calib_tool.h>
#include <calib/io/segment_dataset.h>
#include <calib/lidar_localization.h>

namespace liso {

enum CalibStep {
  Error = 0,
  Start,
  InitializationDone,
  DataAssociationInOdomDone,
  BatchOptimizationDone,
  DataAssociationInRefinementDone,
  RefineDone
};

class LICalibrHelper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LICalibrHelper(const YAML::Node& node);

  void Initialization();

  void DataAssociationInOdom();

  void DataAssociationInLocator();

  void DataAssociationInRefinement();

  void BatchOptimization();

  void Refinement();

  void SaveCalibResult(const std::string& calib_result_file) const;

  void SavePointCloud() const;

 protected:
  void LoadDataset(const YAML::Node& node);

  bool CreateCacheFolder(const std::string& bag_path);

  bool CheckCalibStep(CalibStep desired_step, std::string func_name) const;

  void TrajInitFromSurfel(const TrajectoryEstimatorOptions& options);

  void SetSegmentExtrinsic(int id) {
    calib_param_manager_->p_LinI = p_LinI_backup_.at(id);
    calib_param_manager_->so3_LtoI = so3_LtoI_backup_.at(id);
    calib_param_manager_->UpdateExtrinicParam();
  }

  void ExtrinsicBackup(int id) {
    p_LinI_backup_.at(id) = calib_param_manager_->p_LinI;
    so3_LtoI_backup_.at(id) = calib_param_manager_->so3_LtoI;
  }

  CalibStep calib_step_;

  /// Dataset
  std::string cache_path_;
  std::string cache_path_parent_;
  std::string topic_imu_;
  std::string topic_lidar_;
  std::string bag_name_;
  std::shared_ptr<SegmentDatasetManager> segment_dataset_;

  /// Optimization result
  CalibParamManager::Ptr calib_param_manager_;
  Eigen::aligned_vector<Eigen::Vector3d> p_LinI_backup_;
  Eigen::aligned_vector<SO3d> so3_LtoI_backup_;

  /// Class for each segment
  std::vector<LIDARLocalization::Ptr> locator_vec_;
  std::vector<SurfelAssociation::Ptr> surfel_association_vec_;
  std::vector<std::shared_ptr<ScanUndistortion>> scan_undistortion_vec_;
  std::vector<std::shared_ptr<Trajectory>> trajectory_vec_;

  int iteration_num_;
};

}  // namespace liso

#endif
