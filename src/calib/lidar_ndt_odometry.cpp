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

#include <calib/lidar_ndt_odometry.h>
#include <nav_msgs/Odometry.h>

namespace liso {

LidarNdtOdometry::LidarNdtOdometry(double ndt_resolution,
                                   double ndt_key_frame_downsample)
    : ndt_resolution_(ndt_resolution),
      ndt_key_frame_downsample_(ndt_key_frame_downsample),
      ndt_registration_(new NDTRegistration(ndt_resolution)) {
  RegisterPubSub();
}

void LidarNdtOdometry::RegisterPubSub() {
  pub_global_map_ =
      nh_.advertise<sensor_msgs::PointCloud2>("ndt_odometry/global_map", 1);
  pub_current_cloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("ndt_odometry/cur_cloud", 1);
  pub_laser_odometry_ =
      nh_.advertise<nav_msgs::Odometry>("ndt_odometry/laser_odom", 10);
}

void LidarNdtOdometry::FeedScan(LiDARFeature cur_scan,
                                Eigen::Matrix4d pose_predict,
                                const bool update_map) {
  OdomData odom_cur;
  odom_cur.timestamp = cur_scan.timestamp;
  odom_cur.pose = Eigen::Matrix4d::Identity();

  LiDARFeature result_feature;  // not used
  PosCloud::Ptr scan_in_target(new PosCloud());
  if (global_feature_.full_features->empty()) {
    ndt_registration_->SetInputTarget(cur_scan);
  } else {
    Eigen::Matrix4d T_LtoM_predict = odom_data_.back().pose * pose_predict;
    ndt_registration_->ScanMatch(cur_scan, T_LtoM_predict, result_feature,
                                 odom_cur.pose);
  }

  odom_data_.push_back(odom_cur);

  if (update_map) {
    UpdateKeyScan(cur_scan, odom_cur);
    PublishCloudAndOdom(cur_scan.full_features, true);
  } else {
    PublishCloudAndOdom(cur_scan.full_features);
  }
}

void LidarNdtOdometry::UpdateKeyScan(const LiDARFeature cur_scan,
                                     const OdomData &odom_data) {
  if (CheckKeyScan(odom_data)) {
    PosCloud::Ptr filtered_cloud(new PosCloud);
    PosCloud transform_cloud;
    if (ndt_key_frame_downsample_ > 1e-5) {
      DownsampleCloud(cur_scan.full_features, filtered_cloud,
                      ndt_key_frame_downsample_);
      pcl::transformPointCloud(*filtered_cloud, transform_cloud,
                               odom_data.pose);
    } else {
      pcl::transformPointCloud(*cur_scan.full_features, transform_cloud,
                               odom_data.pose);
    }
    *(global_feature_.full_features) += transform_cloud;
    ndt_registration_->SetInputTarget(global_feature_);
    key_frame_index_.push_back(odom_data_.size() - 1);
  }
}

bool LidarNdtOdometry::CheckKeyScan(const OdomData &odom_data) {
  static Eigen::Vector3d position_last(0, 0, 0);
  static Eigen::Vector3d ypr_last(0, 0, 0);

  Eigen::Vector3d position_now = odom_data.pose.block<3, 1>(0, 3);
  double dist = (position_now - position_last).norm();

  const Eigen::Matrix3d rotation(odom_data.pose.block<3, 3>(0, 0));
  Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
  Eigen::Vector3d delta_angle = ypr - ypr_last;
  for (size_t i = 0; i < 3; i++)
    delta_angle(i) = NormalizeAngle(delta_angle(i));
  delta_angle = delta_angle.cwiseAbs();

  if (key_frame_index_.size() == 0 || dist > 0.2 || delta_angle(0) > 5.0 ||
      delta_angle(1) > 5.0 || delta_angle(2) > 5.0) {
    position_last = position_now;
    ypr_last = ypr;
    return true;
  }
  return false;
}

void LidarNdtOdometry::PublishCloudAndOdom(const PosCloud::Ptr &cur_scan,
                                           bool pub_map) {
  ros::Time time_now = ros::Time::now();

  if (pub_global_map_.getNumSubscribers() > 0 && pub_map) {
    pcl::VoxelGrid<PosPoint> map_filter;
    map_filter.setLeafSize(0.5, 0.5, 0.5);

    PosCloud::Ptr map_cloud_ds(new PosCloud());
    map_filter.setInputCloud(global_feature_.full_features);
    map_filter.filter(*map_cloud_ds);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud_ds, map_msg);
    map_msg.header.frame_id = "/map";
    map_msg.header.stamp = time_now;

    pub_global_map_.publish(map_msg);
  }

  if (pub_laser_odometry_.getNumSubscribers() > 0 && !odom_data_.empty()) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time_now;
    odom_msg.header.frame_id = "/map";

    Eigen::Quaterniond quat =
        Eigen::Quaterniond(odom_data_.back().pose.block<3, 3>(0, 0));
    Eigen::Vector3d pos = odom_data_.back().pose.block<3, 1>(0, 3);
    odom_msg.pose.pose.position.x = pos[0];
    odom_msg.pose.pose.position.y = pos[1];
    odom_msg.pose.pose.position.z = pos[2];
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();
    pub_laser_odometry_.publish(odom_msg);
  }

  if (pub_current_cloud_.getNumSubscribers() > 0 && cur_scan != nullptr) {
    PosCloud::Ptr filtered_cloud(new PosCloud);
    pcl::VoxelGrid<PosPoint> cloud_filter;
    cloud_filter.setLeafSize(0.5, 0.5, 0.5);
    cloud_filter.setInputCloud(cur_scan);
    cloud_filter.filter(*filtered_cloud);

    Eigen::Matrix4d T_cur_to_map = Eigen::Matrix4d::Identity();
    if (!odom_data_.empty()) {
      T_cur_to_map = odom_data_.back().pose;
    }
    PosCloud::Ptr transform_cloud(new PosCloud);
    pcl::transformPointCloud(*filtered_cloud, *transform_cloud, T_cur_to_map);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*transform_cloud, cloud_msg);
    cloud_msg.header.frame_id = "/map";
    cloud_msg.header.stamp = time_now;

    pub_current_cloud_.publish(cloud_msg);
  }
}

void LidarNdtOdometry::ClearOdomData() {
  key_frame_index_.clear();
  odom_data_.clear();
}

}  // namespace liso
