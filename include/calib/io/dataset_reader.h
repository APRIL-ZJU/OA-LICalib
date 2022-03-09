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

#ifndef DATASET_READER_H
#define DATASET_READER_H

/// read rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

/// ros message
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <pcl_conversions/pcl_conversions.h>  /// fromROSMsg toROSMsg

#include <sensor_data/cloud_type.h>
#include <sensor_data/imu_data.h>

#include <sensor_data/lidar_ouster.h>
#include <sensor_data/lidar_rs_16.h>
#include <sensor_data/lidar_vlp_16.h>
#include <sensor_data/lidar_vlp_points.h>

#include <utils/math_utils.h>
#include <utils/eigen_utils.hpp>

#include <Eigen/Core>
#include <fstream>
#include <random>

namespace liso {

namespace IO {

template <typename MsgType, typename MsgTypePtr>
inline bool loadmsg(const std::string bag_path, const std::string topic,
                    std::vector<MsgTypePtr> &msgs, const double bag_start = 0,
                    const double bag_durr = -1) {
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime()
                                         : time_init + ros::Duration(bag_durr);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // Step through the rosbag
  for (const rosbag::MessageInstance &m : view) {
    // Handle IMU measurement
    MsgTypePtr msgPtr = m.instantiate<MsgType>();
    if (msgPtr != NULL) {
      msgs.push_back(msgPtr);
    }
  }

  ROS_INFO("load topic %s", topic.c_str());
  ROS_INFO(
      "time start | end | duration= %.6f | %.6f | %.3f",
      msgs.at(0)->header.stamp.toSec(), msgs.back()->header.stamp.toSec(),
      msgs.back()->header.stamp.toSec() - msgs.at(0)->header.stamp.toSec());

  return true;
}

class LioDataset {
 public:
  LioDataset(LidarModelType lidar_model) : lidar_model_(lidar_model) {}

  void Init() {
    velodyne16_convert_ = nullptr;
    vlp_point_convert_ = nullptr;
    p_robosense_convert_ = nullptr;

    if (lidar_model_ == VLP_16_packet || lidar_model_ == VLP_16_SIMU) {
      velodyne16_convert_ = std::make_shared<Velodyne16>();
      std::cout << "LiDAR model set as VLP_16." << std::endl;
    }  //
    else if (lidar_model_ == VLP_16_points || lidar_model_ == VLP_32E_points) {
      VelodyneType vlp_type = (lidar_model_ == VLP_16_points) ? VLP16 : VLP32E;
      vlp_point_convert_ = std::make_shared<VelodynePoints>(vlp_type);
      std::cout << "LiDAR model set as velodyne_points." << std::endl;
    }  //
    else if (lidar_model_ == Ouster_16_points ||
             lidar_model_ == Ouster_32_points ||
             lidar_model_ == Ouster_64_points ||
             lidar_model_ == Ouster_128_points) {
      OusterRingNo ring_no = OusterRingNo::Ring128;
      if (lidar_model_ == Ouster_16_points)
        ring_no = Ring16;
      else if (lidar_model_ == Ouster_32_points)
        ring_no = Ring32;
      else if (lidar_model_ == Ouster_64_points)
        ring_no = Ring64;
      else if (lidar_model_ == Ouster_128_points)
        ring_no = Ring128;

      ouster_convert_ = std::make_shared<OusterLiDAR>(ring_no);
      lidar_model_ = LidarModelType::Ouster;
      std::cout << "LiDAR model set as Ouster " << int(ring_no) << " points.\n";
    }  //
    else if (lidar_model_ == RS_16) {
      p_robosense_convert_ = std::make_shared<RobosenseCorrection>(
          RobosenseCorrection::ModelType::RS_16);
      std::cout << "LiDAR model set as RS_16." << std::endl;
    }  //
    else {
      std::cout << "LiDAR model " << lidar_model_ << " not support yet."
                << std::endl;
    }
  }

  bool Read(const std::string path, const std::string imu_topic,
            const std::string lidar_topic, const double bag_start = -1.0,
            const double bag_durr = -1.0, const std::string vicon_topic = "") {
    bag_.reset(new rosbag::Bag);
    bag_->open(path, rosbag::bagmode::Read);
    Init();
    rosbag::View view;

    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(lidar_topic);
    if (vicon_topic != "") topics.push_back(vicon_topic);

    if (lidar_model_ == LidarModelType::RS_16) {
      topics.push_back("/rslidar_packets_difop");
    }

    rosbag::View view_full;
    view_full.addQuery(*bag_);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)
                                ? view_full.getEndTime()
                                : time_init + ros::Duration(bag_durr);

    ros::Duration delta_durr = ros::Duration(0.01);
    view.addQuery(*bag_, rosbag::TopicQuery(topics), time_init - delta_durr,
                  time_finish + delta_durr);
    // bag_start_time_ = view_full.getBeginTime().toSec();

    if (lidar_model_ == LidarModelType::RS_16) {
      for (rosbag::MessageInstance const m : view) {
        const std::string &topic = m.getTopic();
        if (topic == "/rslidar_packets_difop") {
          rslidar_msgs::rslidarPacket::ConstPtr difop_msg =
              m.instantiate<rslidar_msgs::rslidarPacket>();
          p_robosense_convert_->processDifop(difop_msg);
          break;
        }
      }
    }

    double first_imu_stamp = -1;
    for (rosbag::MessageInstance const m : view) {
      const std::string &topic = m.getTopic();

      if (lidar_topic == topic) {
        LiDARFeature lidar_feature;
        double timestamp = 0;

        if (lidar_model_ == VLP_16_packet) {
          velodyne_msgs::VelodyneScan::ConstPtr vlp_msg =
              m.instantiate<velodyne_msgs::VelodyneScan>();
          timestamp = vlp_msg->header.stamp.toSec();
          velodyne16_convert_->unpack_scan(vlp_msg, lidar_feature);
        }  //
        else if (lidar_model_ == VLP_16_SIMU) {
          sensor_msgs::PointCloud2::ConstPtr scan_msg =
              m.instantiate<sensor_msgs::PointCloud2>();
          timestamp = scan_msg->header.stamp.toSec();
          velodyne16_convert_->unpack_scan(scan_msg, lidar_feature);
        }  //
        else if (lidar_model_ == VLP_16_points ||
                 lidar_model_ == VLP_32E_points) {
          sensor_msgs::PointCloud2::ConstPtr scan_msg =
              m.instantiate<sensor_msgs::PointCloud2>();
          timestamp = scan_msg->header.stamp.toSec();
          vlp_point_convert_->get_organized_and_raw_cloud(scan_msg,
                                                          lidar_feature);
        }  //
        else if (lidar_model_ == Ouster) {
          sensor_msgs::PointCloud2::ConstPtr scan_msg =
              m.instantiate<sensor_msgs::PointCloud2>();
          timestamp = scan_msg->header.stamp.toSec();
          ouster_convert_->get_organized_and_raw_cloud(scan_msg, lidar_feature);
        }  //
        else if (lidar_model_ == RS_16) {
          rslidar_msgs::rslidarScan::ConstPtr rs_msg =
              m.instantiate<rslidar_msgs::rslidarScan>();
          timestamp = rs_msg->header.stamp.toSec();
          p_robosense_convert_->unpack_scan(rs_msg, lidar_feature);
        }

        lidar_feature.time_max = 0;

        if (first_imu_stamp < 0 || timestamp < first_imu_stamp) continue;

        scan_data_.push_back(lidar_feature);
        scan_timestamps_.emplace_back(timestamp);
      }

      if (imu_topic == topic) {
        sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (first_imu_stamp < 0) {
          first_imu_stamp = imu_msg->header.stamp.toSec();
        }

        imu_data_.emplace_back();
        imu_data_.back().timestamp = imu_msg->header.stamp.toSec();
        imu_data_.back().accel = Eigen::Vector3d(
            imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
            imu_msg->linear_acceleration.z);
        imu_data_.back().gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                                imu_msg->angular_velocity.y,
                                                imu_msg->angular_velocity.z);
        imu_data_.back().orientation =
            Eigen::Quaterniond(imu_msg->orientation.w, imu_msg->orientation.x,
                               imu_msg->orientation.y, imu_msg->orientation.z);

        // imu_data_.back().accel.z() -= 9.8;
      }

      if (vicon_topic == topic) {
        if (m.getDataType() == std::string("nav_msgs/Odometry")) {
          nav_msgs::OdometryConstPtr vicon_msg =
              m.instantiate<nav_msgs::Odometry>();
          vicon_data_.emplace_back();
          vicon_data_.back().timestamp = vicon_msg->header.stamp.toSec();
          auto pose = vicon_msg->pose.pose;
          vicon_data_.back().position = Eigen::Vector3d(
              pose.position.x, pose.position.y, pose.position.z);
          Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z);
          vicon_data_.back().orientation.setQuaternion(quat);
        }
        if (m.getDataType() == std::string("geometry_msgs/TransformStamped")) {
          geometry_msgs::TransformStampedConstPtr vicon_msg =
              m.instantiate<geometry_msgs::TransformStamped>();
          vicon_data_.emplace_back();
          vicon_data_.back().timestamp = vicon_msg->header.stamp.toSec();
          auto pose = vicon_msg->transform;
          vicon_data_.back().position = Eigen::Vector3d(
              pose.translation.x, pose.translation.y, pose.translation.z);
          Eigen::Quaterniond quat(pose.rotation.w, pose.rotation.x,
                                  pose.rotation.y, pose.rotation.z);
          vicon_data_.back().orientation.setQuaternion(quat);
        }
      }
    }

    std::cout << lidar_topic << ": " << scan_data_.size() << std::endl;
    std::cout << imu_topic << ": " << imu_data_.size() << std::endl;
    if (!vicon_data_.empty())
      std::cout << vicon_topic << ": " << vicon_data_.size() << std::endl;
  }

  void AdjustIMUViconData() {
    assert(imu_data_.size() > 0 && "No IMU data. Check your bag and imu topic");
    assert(vicon_data_.size() > 0 &&
           "No vicon data. Check your bag and vicon topic");

    start_time_ =
        std::min(imu_data_.front().timestamp, vicon_data_.front().timestamp);
    end_time_ =
        std::max(imu_data_.back().timestamp, vicon_data_.back().timestamp);

    for (size_t i = 0; i < imu_data_.size(); i++) {
      imu_data_[i].timestamp -= start_time_;
    }

    for (size_t i = 0; i < vicon_data_.size(); i++) {
      vicon_data_[i].timestamp -= start_time_;
    }
  }

  void AdjustDatasetTime() {
    assert(imu_data_.size() > 0 && "No IMU data. Check your bag and imu topic");
    assert(scan_data_.size() > 0 &&
           "No scan data. Check your bag and lidar topic");

    assert(scan_timestamps_.front() < imu_data_.back().timestamp &&
           scan_timestamps_.back() > imu_data_.front().timestamp &&
           "Unvalid dataset. Check your dataset.. ");

    start_time_ =
        std::min(scan_timestamps_.front(), imu_data_.front().timestamp);
    end_time_ = std::max(scan_timestamps_.back(), imu_data_.back().timestamp);
    std::cout << "start_time set as " << start_time_ << std::endl;

    for (size_t i = 0; i < imu_data_.size(); i++) {
      imu_data_[i].timestamp -= start_time_;
    }

    for (size_t j = 0; j < scan_data_.size(); j++) {
      scan_data_[j].timestamp -= start_time_;
      scan_timestamps_.at(j) -= start_time_;
      for (size_t i = 0; i < scan_data_[j].full_features->size(); i++) {
        scan_data_[j].full_features->points[i].timestamp -= start_time_;
      }
    }

    if (!vicon_data_.empty()) {
      for (size_t i = 0; i < vicon_data_.size(); i++) {
        vicon_data_[i].timestamp -= start_time_;
      }
    }

    std::cout << "IMU timestamp from : " << imu_data_.front().timestamp
              << " to " << imu_data_.back().timestamp << std::endl;
    std::cout << "scan timestamp from : " << scan_data_.front().timestamp
              << " to "
              << scan_data_.back().full_features->points.back().timestamp
              << std::endl;
  }

  void AddSimulationTimeoffset(double added_timeoffset) {
    for (size_t j = 0; j < scan_data_.size(); j++) {
      scan_data_[j].timestamp += added_timeoffset;
      for (size_t i = 0; i < scan_data_[j].full_features->size(); i++) {
        scan_data_[j].full_features->points[i].timestamp += added_timeoffset;
      }
    }
    for (size_t i = 0; i < scan_timestamps_.size(); i++) {
      scan_timestamps_[i] += added_timeoffset;
    }
  }

  void Reset() {
    imu_data_.clear();
    scan_data_.clear();
    scan_timestamps_.clear();
    vicon_data_.clear();
  }

  double get_start_time() const { return start_time_; }

  double get_end_time() const { return end_time_; }

  const std::vector<double> &get_scan_timestamps() const {
    return scan_timestamps_;
  }

  const Eigen::aligned_vector<IMUData> &get_imu_data() const {
    return imu_data_;
  }

  const Eigen::aligned_vector<PoseData> &get_vicon_data() const {
    return vicon_data_;
  }

  const std::vector<LiDARFeature> &get_scan_data() const { return scan_data_; }

 public:
  std::shared_ptr<rosbag::Bag> bag_;

  Eigen::aligned_vector<IMUData> imu_data_;

  std::vector<LiDARFeature> scan_data_;

  std::vector<double> scan_timestamps_;

  Eigen::aligned_vector<PoseData> vicon_data_;

  double start_time_;
  double end_time_;

  Velodyne16::Ptr velodyne16_convert_;
  VelodynePoints::Ptr vlp_point_convert_;
  OusterLiDAR::Ptr ouster_convert_;
  RobosenseCorrection::Ptr p_robosense_convert_;

  LidarModelType lidar_model_;
};

}  // namespace IO
}  // namespace liso

#endif  // DATASET_READER_H
