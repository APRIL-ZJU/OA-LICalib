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

#include <calib/lidar_localization.h>

namespace liso {

LIDARLocalization::LIDARLocalization(double ndt_resolution,
                                     double ndt_key_frame_downsample,
                                     double map_downsample_size,
                                     const Eigen::Matrix4d init_pose)
    : init_pose_(init_pose),
      ndt_resolution_(ndt_resolution),
      ndt_key_frame_downsample_(ndt_key_frame_downsample),
      ndt_registration_(new NDTRegistration(ndt_resolution)) {
  RegisterPubSub();
  map_filter_.setLeafSize(map_downsample_size, map_downsample_size,
                          map_downsample_size);

  cloud_filter_.setLeafSize(ndt_key_frame_downsample, ndt_key_frame_downsample,
                            ndt_key_frame_downsample);
}

void LIDARLocalization::RegisterPubSub() {
  pub_global_map_ =
      nh_.advertise<sensor_msgs::PointCloud2>("localization/global_map", 1);
  pub_current_cloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("localization/cur_cloud", 1);
  pub_laser_odometry_ =
      nh_.advertise<nav_msgs::Odometry>("localization/laser_odom", 10);
}

void LIDARLocalization::SetPriorMap(std::string map_pcd_path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "[SetPriorMap] loading prior map as " << map_pcd_path << "\n";

  //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_pcd_path, *cloud) == -1) {
    ROS_WARN("Couldn't read file %s \n", map_pcd_path.c_str());
    return;
  }

  LiDARFeature map_cloud;
  map_cloud.timestamp = 0;

  map_cloud.full_features->reserve(cloud->size());
  for (auto const& v : cloud->points) {
    PosPoint p;
    p.x = v.x;
    p.y = v.y;
    p.z = v.z;
    p.timestamp = 0;
    map_cloud.full_features->push_back(p);
  }

  SetPriorMap(map_cloud);

  // pcl::io::savePCDFileBinaryCompressed(map_pcd_path + "-ds.pcd",
  //                                      *prior_map_.full_features);
}

void LIDARLocalization::SetPriorMap(const LiDARFeature& map_cloud) {
  PosCloud::Ptr map_cloud_ds(new PosCloud());
  map_filter_.setInputCloud(map_cloud.full_features);
  map_filter_.filter(*map_cloud_ds);

  prior_map_.timestamp = map_cloud.timestamp;
  prior_map_.full_features = map_cloud_ds;

  ndt_registration_->SetInputTarget(prior_map_);

  // prepare prior map msg
  pcl::toROSMsg(*map_cloud_ds, map_msg_);
  map_msg_.header.frame_id = "/map";
  map_msg_.header.stamp = ros::Time::now();

  std::cout << "[SetPriorMap] prior map size " << map_cloud_ds->size() << "\n";
}

void LIDARLocalization::FeedScan(const LiDARFeature& cur_scan,
                                 const Eigen::Matrix4d& pose_predict,
                                 bool pridict_is_relative) {
  OdomData odom_cur;
  odom_cur.timestamp = cur_scan.timestamp;
  odom_cur.pose = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d T_LtoM_predict;
  if (pridict_is_relative) {
    if (odom_data_.empty())
      T_LtoM_predict = init_pose_;
    else {
      int idx = odom_data_.size();
      if (idx >= 2) {
        const Eigen::Matrix4d& T0 = odom_data_[idx - 2].pose;
        const Eigen::Matrix4d& T1 = odom_data_[idx - 1].pose;

        Eigen::Quaterniond q_incre = Eigen::Quaterniond(
            T0.block<3, 3>(0, 0).transpose() * T1.block<3, 3>(0, 0));
        Eigen::Vector3d p_incre = T0.block<3, 3>(0, 0).transpose() *
                                  (T1.block<3, 1>(0, 3) - T0.block<3, 1>(0, 3));

        Eigen::Quaterniond q_predict =
            Eigen::Quaterniond(T1.block<3, 3>(0, 0)) * q_incre;
        q_predict.normalize();
        T_LtoM_predict.block<3, 3>(0, 0) = q_predict.toRotationMatrix();

        T_LtoM_predict.block<3, 1>(0, 3) =
            T1.block<3, 3>(0, 0) * p_incre + T1.block<3, 1>(0, 3);
      } else
        T_LtoM_predict = odom_data_.back().pose * pose_predict;
    }
  } else {
    T_LtoM_predict = pose_predict;
  }

  LiDARFeature result_feature;  // not used
  ndt_registration_->ScanMatch(cur_scan, T_LtoM_predict, result_feature,
                               odom_cur.pose);

  odom_data_.push_back(odom_cur);
  if (CheckKeyScan(odom_cur)) {
    key_frame_index_.push_back(odom_data_.size() - 1);
  }
}

bool LIDARLocalization::CheckKeyScan(const OdomData& odom_data) {
  // pose of last key frame
  static Eigen::Vector3d position_last(0, 0, 0);
  static Eigen::Vector3d ypr_last(0, 0, 0);

  // position change
  Eigen::Vector3d position_now = odom_data.pose.block<3, 1>(0, 3);
  double dist = (position_now - position_last).norm();

  // roation change
  const Eigen::Matrix3d rotation(odom_data.pose.block<3, 3>(0, 0));
  Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
  Eigen::Vector3d delta_angle = ypr - ypr_last;
  for (size_t i = 0; i < 3; i++)
    delta_angle(i) = NormalizeAngle(delta_angle(i));
  delta_angle = delta_angle.cwiseAbs();

  bool is_key_frame = false;
  if (key_frame_index_.size() == 0 || dist > 0.2 || delta_angle(0) > 5.0 ||
      delta_angle(1) > 5.0 || delta_angle(2) > 5.0) {
    is_key_frame = true;
    position_last = position_now;
    ypr_last = ypr;
  }

  return is_key_frame;
}

void LIDARLocalization::CaculateGlobalMapAndOdom(
    const std::map<double, LiDARFeature>& scan_data) {
  assert(odom_data_.size() == scan_data.size() &&
         "odom_data size diff from scan_data");
  Eigen::Matrix3d R0_inv =
      odom_data_.front().pose.block<3, 3>(0, 0).transpose();
  Eigen::Vector3d P0 = R0_inv * (-odom_data_.front().pose.block<3, 1>(0, 3));

  new_map_.full_features->clear();
  new_map_.timestamp = scan_data.begin()->first;
  new_map_.full_features->is_dense = false;

  int k = 0;
  auto iter_scan = scan_data.begin();
  int key_frame_idx = 0;
  while (iter_scan != scan_data.end()) {
    Eigen::Matrix3d Rk = odom_data_[k].pose.block<3, 3>(0, 0);
    Eigen::Vector3d pk = odom_data_[k].pose.block<3, 1>(0, 3);
    odom_data_[k].pose.block<3, 3>(0, 0) = R0_inv * Rk;
    odom_data_[k].pose.block<3, 1>(0, 3) = R0_inv * pk + P0;

    if (key_frame_index_[key_frame_idx] == k) {
      key_frame_idx++;

      PosCloud::Ptr filtered_cloud(new PosCloud);
      cloud_filter_.setInputCloud(iter_scan->second.full_features);
      cloud_filter_.filter(*filtered_cloud);

      PosCloud transform_cloud;
      pcl::transformPointCloud(*filtered_cloud, transform_cloud,
                               odom_data_[k].pose);

      *(new_map_.full_features) += transform_cloud;
    }
    k++;
    iter_scan++;
  }
  std::cout << "global map from locator size: "
            << new_map_.full_features->size() << "\n";
}

void LIDARLocalization::PublishCloudAndOdom(const PosCloud::Ptr& cur_scan) {
  ros::Time time_now = ros::Time::now();

  static bool has_pub_map = false;
  if (pub_global_map_.getNumSubscribers() > 0 && !has_pub_map) {
    has_pub_map = true;
    map_msg_.header.stamp = time_now;
    pub_global_map_.publish(map_msg_);
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
    cloud_filter_.setInputCloud(cur_scan);
    cloud_filter_.filter(*filtered_cloud);

    Eigen::Matrix4d T_cur_to_map;
    if (odom_data_.empty())
      T_cur_to_map = init_pose_;
    else
      T_cur_to_map = odom_data_.back().pose;
    PosCloud::Ptr transform_cloud(new PosCloud);
    pcl::transformPointCloud(*filtered_cloud, *transform_cloud, T_cur_to_map);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*transform_cloud, cloud_msg);
    cloud_msg.header.frame_id = "/map";
    cloud_msg.header.stamp = time_now;

    pub_current_cloud_.publish(cloud_msg);
  }
}

void LIDARLocalization::ClearOdomData() {
  key_frame_index_.clear();
  odom_data_.clear();
}

}  // namespace liso
