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

#ifndef TRAJECTORY_VIEWER_H
#define TRAJECTORY_VIEWER_H

#include <sensor_data/imu_data.h>
#include <trajectory/se3_trajectory.h>

#include <eigen_conversions/eigen_msg.h>
#include <oa_licalib/imu_array.h>
#include <oa_licalib/pose_array.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>

namespace liso {

namespace publisher {
extern ros::Publisher pub_trajectory_raw_;
extern ros::Publisher pub_trajectory_est_;
extern ros::Publisher pub_imu_raw_array_;
extern ros::Publisher pub_imu_est_array_;
extern ros::Publisher pub_target_cloud_;
extern ros::Publisher pub_source_cloud_;

extern ros::Publisher pub_spline_trajectory_;
extern ros::Publisher pub_lidar_trajectory_;

void SetPublisher(ros::NodeHandle &nh);

};  // namespace publisher

namespace TrajectoryViewer {
inline void PublishIMUData(std::shared_ptr<Trajectory> trajectory,
                           const Eigen::aligned_vector<IMUData> &imu_data) {
  if (publisher::pub_imu_raw_array_.getNumSubscribers() == 0 &&
      publisher::pub_imu_est_array_.getNumSubscribers() == 0)
    return;

  oa_licalib::imu_array imu_array_raw;
  oa_licalib::imu_array imu_array_est;

  for (auto const &v : imu_data) {
    if (!trajectory->GetTrajQuality(v.timestamp)) {
      continue;
    }
    geometry_msgs::Vector3 gyro, accel;
    tf::vectorEigenToMsg(v.gyro, gyro);
    tf::vectorEigenToMsg(v.accel, accel);
    imu_array_raw.timestamps.push_back(v.timestamp);
    imu_array_raw.angular_velocities.push_back(gyro);
    imu_array_raw.linear_accelerations.push_back(accel);

    auto const param = trajectory->GetTrajParam();

    Eigen::Vector3d w_b =
        trajectory->rotVelBody(v.timestamp) + param->gyro_bias;
    Eigen::Vector3d a_w = trajectory->transAccelWorld(v.timestamp);
    SE3d pose = trajectory->pose(v.timestamp);
    Eigen::Vector3d a_b =
        pose.so3().inverse() * (a_w + param->gravity) + param->acce_bias;

    geometry_msgs::Vector3 gyro2, accel2;
    tf::vectorEigenToMsg(w_b, gyro2);
    tf::vectorEigenToMsg(a_b, accel2);
    imu_array_est.timestamps.push_back(v.timestamp);
    imu_array_est.angular_velocities.push_back(gyro2);
    imu_array_est.linear_accelerations.push_back(accel2);
  }
  imu_array_raw.header.stamp = ros::Time::now();
  imu_array_raw.header.frame_id = "/imu";

  imu_array_est.header = imu_array_raw.header;

  publisher::pub_imu_raw_array_.publish(imu_array_raw);
  publisher::pub_imu_est_array_.publish(imu_array_est);
}

inline void PublishViconData(
    std::shared_ptr<Trajectory> trajectory,
    const Eigen::aligned_vector<PoseData> &vicon_data) {
  if (publisher::pub_trajectory_raw_.getNumSubscribers() == 0 &&
      publisher::pub_trajectory_est_.getNumSubscribers() == 0)
    return;

  oa_licalib::pose_array vicon_path_raw;
  oa_licalib::pose_array vicon_path_est;

  for (auto const &v : vicon_data) {
    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation;

    // raw data
    tf::vectorEigenToMsg(v.position, position);
    tf::quaternionEigenToMsg(v.orientation.unit_quaternion(), orientation);

    vicon_path_raw.timestamps.push_back(v.timestamp);
    vicon_path_raw.positions.push_back(position);
    vicon_path_raw.orientations.push_back(orientation);

    // estiamted pose
    SE3d pose;
    if (!trajectory->GetLidarPose(v.timestamp, pose)) continue;
    tf::vectorEigenToMsg(pose.translation(), position);
    tf::quaternionEigenToMsg(pose.unit_quaternion(), orientation);
    vicon_path_est.timestamps.push_back(v.timestamp);
    vicon_path_est.positions.push_back(position);
    vicon_path_est.orientations.push_back(orientation);
  }

  vicon_path_raw.header.frame_id = "/map";
  vicon_path_raw.header.stamp = ros::Time::now();
  vicon_path_est.header = vicon_path_raw.header;

  publisher::pub_trajectory_raw_.publish(vicon_path_raw);
  publisher::pub_trajectory_est_.publish(vicon_path_est);
}

inline void PublishViconData(
    const Eigen::aligned_vector<PoseData> &vicon_est,
    const Eigen::aligned_vector<PoseData> &vicon_data) {
  if (publisher::pub_trajectory_raw_.getNumSubscribers() == 0 &&
      publisher::pub_trajectory_est_.getNumSubscribers() == 0)
    return;

  oa_licalib::pose_array vicon_path_raw;
  oa_licalib::pose_array vicon_path_est;

  for (auto const &v : vicon_data) {
    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation;

    // raw data
    tf::vectorEigenToMsg(v.position, position);
    tf::quaternionEigenToMsg(v.orientation.unit_quaternion(), orientation);

    vicon_path_raw.timestamps.push_back(v.timestamp);
    vicon_path_raw.positions.push_back(position);
    vicon_path_raw.orientations.push_back(orientation);
  }

  for (auto const &v : vicon_est) {
    // estiamted pose
    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation;

    tf::vectorEigenToMsg(v.position, position);
    tf::quaternionEigenToMsg(v.orientation.unit_quaternion(), orientation);

    vicon_path_est.timestamps.push_back(v.timestamp);
    vicon_path_est.positions.push_back(position);
    vicon_path_est.orientations.push_back(orientation);
  }

  vicon_path_raw.header.frame_id = "/map";
  vicon_path_raw.header.stamp = ros::Time::now();
  vicon_path_est.header = vicon_path_raw.header;

  publisher::pub_trajectory_raw_.publish(vicon_path_raw);
  publisher::pub_trajectory_est_.publish(vicon_path_est);
}

inline void PublishIMUOrientationData(
    std::shared_ptr<Trajectory> trajectory,
    const Eigen::aligned_vector<PoseData> &orientation_data) {
  oa_licalib::pose_array imu_ori_path_raw;
  oa_licalib::pose_array imu_ori_path_est;

  for (auto const &v : orientation_data) {
    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation;

    // raw data
    tf::vectorEigenToMsg(v.position, position);
    tf::quaternionEigenToMsg(v.orientation.unit_quaternion(), orientation);

    imu_ori_path_raw.timestamps.push_back(v.timestamp);
    imu_ori_path_raw.positions.push_back(position);
    imu_ori_path_raw.orientations.push_back(orientation);

    // estiamted pose
    SE3d pose = trajectory->pose(v.timestamp);
    tf::vectorEigenToMsg(pose.translation(), position);
    tf::quaternionEigenToMsg(pose.unit_quaternion(), orientation);
    imu_ori_path_est.timestamps.push_back(v.timestamp);
    imu_ori_path_est.positions.push_back(position);
    imu_ori_path_est.orientations.push_back(orientation);
  }

  imu_ori_path_raw.header.frame_id = "/map";
  imu_ori_path_raw.header.stamp = ros::Time::now();
  imu_ori_path_est.header = imu_ori_path_raw.header;

  publisher::pub_trajectory_raw_.publish(imu_ori_path_raw);
  publisher::pub_trajectory_est_.publish(imu_ori_path_est);
}

inline void PublishLoamCorrespondence(
    std::shared_ptr<Trajectory> trajectory,
    const Eigen::aligned_vector<PointCorrespondence> &point_measurement) {
  if (publisher::pub_source_cloud_.getNumSubscribers() == 0 &&
      publisher::pub_target_cloud_.getNumSubscribers() == 0)
    return;

  VPointCloud target_cloud, source_cloud;
  for (const PointCorrespondence &cor : point_measurement) {
    if (cor.t_map < trajectory->minTime() ||
        cor.t_point < trajectory->minTime()) {
      std::cout << RED << "[PublishLoamCorrespondence] skip " << cor.t_map
                << "; " << cor.t_point << RESET << std::endl;
      continue;
    }

    // SE3d T_MtoG = trajectory->GetLidarPose(cor.t_map);
    SE3d T_LktoG = trajectory->GetLidarPose(cor.t_point);

    // Eigen::Vector3d p_inM = T_MtoG.inverse() * T_LktoG * cor.point;
    Eigen::Vector3d p_inM = T_LktoG * cor.point;

    VPoint p_target, p_source;
    Eigen::Vector3d p_intersect;
    if (cor.geo_type == GeometryType::Plane) {
      double dist = p_inM.dot(cor.geo_plane.head(3)) + cor.geo_plane[3];
      p_intersect = p_inM + dist * cor.geo_plane.head(3);

      p_target.intensity = 100;
      p_source.intensity = 100;
    } else {
      double t = (p_inM - cor.geo_point).dot(cor.geo_normal);
      p_intersect = cor.geo_point - t * cor.geo_normal;

      p_target.intensity = 50;
      p_source.intensity = 50;
    }

    p_target.x = p_intersect[0];
    p_target.y = p_intersect[1];
    p_target.z = p_intersect[2];
    target_cloud.push_back(p_target);
    p_source.x = p_inM[0];
    p_source.y = p_inM[1];
    p_source.z = p_inM[2];
    source_cloud.push_back(p_source);
  }

  sensor_msgs::PointCloud2 target_msg, source_msg;
  pcl::toROSMsg(target_cloud, target_msg);
  pcl::toROSMsg(source_cloud, source_msg);

  target_msg.header.stamp = ros::Time::now();
  target_msg.header.frame_id = "/map";
  source_msg.header = target_msg.header;

  publisher::pub_target_cloud_.publish(target_msg);
  publisher::pub_source_cloud_.publish(source_msg);
}

inline void PublishSplineTrajectory(std::shared_ptr<Trajectory> trajectory,
                                    double min_time, double max_time,
                                    double dt) {
  if (min_time < trajectory->minTime()) min_time = trajectory->minTime();
  if (max_time > trajectory->maxTime()) max_time = trajectory->maxTime();

  if (publisher::pub_spline_trajectory_.getNumSubscribers() != 0) {
    ros::Time t_temp;
    std::vector<geometry_msgs::PoseStamped> poses_geo;
    for (double t = min_time; t < max_time; t += dt) {
      SE3d pose = trajectory->pose(t);
      geometry_msgs::PoseStamped poseIinG;
      poseIinG.header.stamp = t_temp.fromSec(t);
      poseIinG.header.frame_id = "/map";
      tf::pointEigenToMsg(pose.translation(), poseIinG.pose.position);
      tf::quaternionEigenToMsg(pose.unit_quaternion(),
                               poseIinG.pose.orientation);
      poses_geo.push_back(poseIinG);
    }
    ros::Time time_now = ros::Time::now();
    nav_msgs::Path traj_path;
    traj_path.header.stamp = time_now;
    traj_path.header.frame_id = "/map";
    traj_path.poses = poses_geo;

    publisher::pub_spline_trajectory_.publish(traj_path);
  }

  if (publisher::pub_lidar_trajectory_.getNumSubscribers() != 0) {
    ros::Time t_temp;
    std::vector<geometry_msgs::PoseStamped> poses_geo;
    for (double t = min_time; t < max_time; t += dt) {
      SE3d pose;
      if (trajectory->GetLidarPose(t, pose)) {
        geometry_msgs::PoseStamped poseIinG;
        poseIinG.header.stamp = t_temp.fromSec(t);
        poseIinG.header.frame_id = "/map";
        tf::pointEigenToMsg(pose.translation(), poseIinG.pose.position);
        tf::quaternionEigenToMsg(pose.unit_quaternion(),
                                 poseIinG.pose.orientation);
        poses_geo.push_back(poseIinG);
      }
    }
    ros::Time time_now = ros::Time::now();
    nav_msgs::Path traj_path;
    traj_path.header.stamp = time_now;
    traj_path.header.frame_id = "/map";
    traj_path.poses = poses_geo;

    publisher::pub_lidar_trajectory_.publish(traj_path);
  }
}
}  // namespace TrajectoryViewer

}  // namespace liso

#endif
