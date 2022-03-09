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

#ifndef SE3_TRAJECTORY_H
#define SE3_TRAJECTORY_H

#include <basalt/spline/se3_spline.h>
#include <sensor_data/calibration.h>
#include <sensor_data/cloud_type.h>
#include <sophus/se3.hpp>
#include <string>

namespace liso {

class Trajectory : public basalt::Se3Spline<SplineOrder, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Trajectory> Ptr;

  Trajectory(double time_interval, double start_time = 0, size_t segment_id = 0)
      : basalt::Se3Spline<SplineOrder, double>(time_interval, start_time),
        // calib_param_(std::make_shared<CalibParamManager>()),
        segment_id_(segment_id) {
    this->extendKnotsTo(start_time, SO3d(Eigen::Quaterniond::Identity()),
                        Eigen::Vector3d(0, 0, 0));
  }

  void SetCalibParam(std::shared_ptr<CalibParamManager> calib_param) {
    //    calib_param_ = std::move(calib_param);
    calib_param_ = calib_param;
  }

  const std::shared_ptr<CalibParamManager> GetCalibParam() const {
    return calib_param_;
  }

  const size_t SegmentID() const { return segment_id_; }

  SegmentCalibParam *GetTrajParam() {
    return &calib_param_->segment_param[segment_id_];
  }

  const SegmentCalibParam *GetTrajParam() const {
    return &calib_param_->segment_param[segment_id_];
  }

  bool GetTrajQuality(const double timestamp) const {
    if (timestamp < this->minTime() || timestamp >= this->maxTime())
      return false;
    else
      return true;
  }

  bool GetLiDARTrajQuality(const double timestamp) const {
    double t_lidar = timestamp + this->GetTrajParam()->time_offset;

    return GetTrajQuality(t_lidar);
  }

  SE3d GetLidarPose(const double timestamp) const;

  bool GetLidarPose(const double timestamp, SE3d &lidar_pose);

  void UndistortScan(const PosCloud &scan_raw, const double target_timestamp,
                     PosCloud &scan_in_target) const;

  bool EvaluateLidarRelativeRotation(double lidar_time1, double lidar_time2,
                                     Eigen::Quaterniond &q_L2toL1);

  void SaveTrajectoryControlPoints(std::string path);

  bool LoadTrajectoryControlPoints(std::string path);

  void TrajectoryToTUMTxt(std::string file_path, double relative_start_time = 0,
                          double relative_end_time = 0, int iteration_num = 0,
                          double dt = 0.02) const;

  void TrajectoryToTUMTxt2(std::string file_path,
                           double relative_start_time = 0,
                           double relative_end_time = 0,
                           double dt = 0.02) const;

 private:
  CalibParamManager::Ptr calib_param_;

  size_t segment_id_;
};

}  // namespace liso
#endif
