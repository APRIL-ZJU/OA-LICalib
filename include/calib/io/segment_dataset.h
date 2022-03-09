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

#ifndef SEGMENT_DATASET_H
#define SEGMENT_DATASET_H

#include <calib/io/dataset_reader.h>

namespace liso {

struct SegmentDataset {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::aligned_vector<IMUData> imu_data;
  std::vector<LiDARFeature> scan_data;
  std::vector<double> scan_timestamps;

  Eigen::aligned_vector<PoseData> vicon_data;

  double start_time;
  double end_time;

  // double bag_start_time;
};

class SegmentDatasetManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SegmentDatasetManager(const YAML::Node& node,
                        const LidarModelType& lidar_model) {
    int segment_num = node["segment_num"].as<int>();

    const YAML::Node& segment_node = node["selected_segment"];
    for (int i = 0; i < segment_num; i++) {
      std::string path_bag = segment_node[i]["path_bag"].as<std::string>();
      std::pair<double, double> segment_t;
      segment_t.first = segment_node[i]["start_time"].as<double>();
      segment_t.second = segment_node[i]["end_time"].as<double>();

      path_bag_vec_.push_back(path_bag);
      segment_timestamp_.push_back(segment_t);
    }

    std::string topic_imu = node["topic_imu"].as<std::string>();
    std::string topic_lidar = node["topic_lidar"].as<std::string>();

    for (int id = 0; id < segment_num; ++id) {
      auto v = &segment_timestamp_.at(id);
      double bag_start = v->first;
      double bag_durr = v->second - v->first;
      std::string bag_path = path_bag_vec_.at(id);

      std::cout << "Load dataset from " << bag_path << std::endl;
      std::cout << "bag start | bag durr : " << bag_start << " | " << bag_durr
                << std::endl;

      std::shared_ptr<liso::IO::LioDataset> dataset_reader;
      dataset_reader = std::make_shared<liso::IO::LioDataset>(lidar_model);
      dataset_reader->Read(bag_path, topic_imu, topic_lidar, bag_start,
                           bag_durr);
      dataset_reader->AdjustDatasetTime();

      AddSegmentData(dataset_reader);
    }

    std::cout << "SegmentDataset\n";
    for (size_t i = 0; i < segment_num; ++i) {
      std::cout << "\t-segment [" << i
                << "] : imu/lidar = " << segment_dataset_vec_[i].imu_data.size()
                << "/" << segment_dataset_vec_[i].scan_timestamps.size()
                << "; start/end_time = " << segment_dataset_vec_[i].start_time
                << "/" << segment_dataset_vec_[i].end_time << "\n";
    }
  }

  void AddSegmentData(
      const std::shared_ptr<liso::IO::LioDataset>& lio_dataset) {
    segment_dataset_vec_.emplace_back();
    segment_dataset_vec_.back().imu_data = lio_dataset->imu_data_;
    segment_dataset_vec_.back().scan_data = lio_dataset->scan_data_;
    segment_dataset_vec_.back().scan_timestamps = lio_dataset->scan_timestamps_;
    segment_dataset_vec_.back().vicon_data = lio_dataset->vicon_data_;
    segment_dataset_vec_.back().start_time = lio_dataset->start_time_;
    segment_dataset_vec_.back().end_time = lio_dataset->end_time_;
    // segment_dataset_vec_.back().bag_start_time =
    // lio_dataset->bag_start_time_;
  }

  bool SegmentValidationCheck(
      const Eigen::aligned_vector<Eigen::Vector3d>& gyro_vec) const {
    if (gyro_vec.size() < 5) return false;

    Eigen::Vector3d min_gyro = gyro_vec.at(0);
    Eigen::Vector3d max_gyro = gyro_vec.at(0);
    for (auto& gyro : gyro_vec) {
      for (int i = 0; i < 3; ++i) {
        min_gyro[i] = gyro[i] < min_gyro[i] ? gyro[i] : min_gyro[i];
        max_gyro[i] = gyro[i] > max_gyro[i] ? gyro[i] : max_gyro[i];
      }
    }

    if ((max_gyro - min_gyro).maxCoeff() > 0.05) return true;

    return false;
  }

  const size_t SegmentNum() const { return segment_timestamp_.size(); }

  const double GetStartTime(size_t segment_id) const {
    return segment_dataset_vec_.at(segment_id).start_time;
  }

  const double GetEndTime(size_t segment_id) const {
    return segment_dataset_vec_.at(segment_id).end_time;
  }

  //  const double GetBagStartTime(size_t segment_id) const {
  //    return segment_dataset_vec_.at(segment_id).bag_start_time;
  //  }

  const std::vector<double>& GetScanTimestamps(size_t segment_id) const {
    return segment_dataset_vec_.at(segment_id).scan_timestamps;
  }

  const std::vector<LiDARFeature>& GetScanData(size_t segment_id) const {
    return segment_dataset_vec_.at(segment_id).scan_data;
  }

  const Eigen::aligned_vector<IMUData>& GetImuData(size_t segment_id) const {
    return segment_dataset_vec_.at(segment_id).imu_data;
  }

  const Eigen::aligned_vector<PoseData>& GetViconData(size_t segment_id) const {
    return segment_dataset_vec_.at(segment_id).vicon_data;
  }

  const std::vector<std::pair<double, double>>& GetSegmentTimestamp() const {
    return segment_timestamp_;
  }

  const std::vector<std::string>& GetSegmentBagPath() const {
    return path_bag_vec_;
  }

 private:
  std::vector<std::pair<double, double>> segment_timestamp_;

  std::vector<std::string> path_bag_vec_;

  // raw data
  std::vector<SegmentDataset> segment_dataset_vec_;
};

}  // namespace liso

#endif  // DATASET_READER_H
