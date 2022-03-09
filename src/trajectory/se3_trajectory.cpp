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

#include <trajectory/se3_trajectory.h>

namespace liso {

SE3d Trajectory::GetLidarPose(const double timestamp) const {
  double t = timestamp + this->GetTrajParam()->time_offset;
  if (t < this->minTime()) {
    t = this->minTime();
  } else if (t >= this->maxTime()) {
    t = this->maxTime() - 1e-9;
  }

  SE3d pose_I_to_G = this->pose(t);
  SE3d pose_L_to_G = pose_I_to_G * calib_param_->se3_LtoI;
  return pose_L_to_G;
}

bool Trajectory::GetLidarPose(const double timestamp, SE3d &lidar_pose) {
  double t = timestamp + this->GetTrajParam()->time_offset;
  if (t < this->minTime() || t >= this->maxTime()) return false;

  SE3d pose_I_to_G = this->pose(t);
  SE3d pose_L_to_G = pose_I_to_G * calib_param_->se3_LtoI;
  lidar_pose = pose_L_to_G;
  return true;
}

void Trajectory::UndistortScan(const PosCloud &scan_raw,
                               const double target_timestamp,
                               PosCloud &scan_in_target) const {
  scan_in_target.header = scan_raw.header;
  scan_in_target.resize(scan_raw.size());
  scan_in_target.is_dense = true;

  SE3d pose_G_to_target = GetLidarPose(target_timestamp).inverse();

  std::size_t cnt = 0;
  for (auto const &raw_p : scan_raw.points) {
    if (pcl_isnan(raw_p.x)) {
      scan_in_target.is_dense = false;
      std::cout << RED << "[UndistortScan] input cloud exists NAN point\n"
                << RESET;
      continue;
    }
    SE3d pose_Lk_to_G = GetLidarPose(raw_p.timestamp);

    Eigen::Vector3d p_Lk(raw_p.x, raw_p.y, raw_p.z);
    Eigen::Vector3d point_out;
    point_out = pose_G_to_target * pose_Lk_to_G * p_Lk;

    PosPoint point;
    point.x = point_out(0);
    point.y = point_out(1);
    point.z = point_out(2);
    point.timestamp = raw_p.timestamp;

    scan_in_target[cnt++] = point;
  }
}

bool Trajectory::EvaluateLidarRelativeRotation(double lidar_time1,
                                               double lidar_time2,
                                               Eigen::Quaterniond &q_L2toL1) {
  assert(lidar_time1 <= lidar_time2 &&
         "[EvaluateRelativeRotation] : lidar_time1 > lidar_time2");
  if (lidar_time1 < this->minTime() || lidar_time2 > this->maxTime())
    return false;

  SE3d pose_I1_to_G = this->pose(lidar_time1);
  SE3d pose_I2_to_G = this->pose(lidar_time2);
  Eigen::Quaterniond q1 = pose_I1_to_G.unit_quaternion();
  Eigen::Quaterniond q2 = pose_I2_to_G.unit_quaternion();

  Eigen::Quaterniond q_I2toI1 = q1.conjugate() * q2;
  q_L2toL1 = calib_param_->q_LtoI.conjugate() * q_I2toI1 * calib_param_->q_LtoI;
  return true;
}

void Trajectory::SaveTrajectoryControlPoints(std::string path) {
  std::ofstream outfile;
  outfile.open(path);

  size_t NumKnots = this->numKnots();
  for (size_t i = 0; i < NumKnots; i++) {
    Eigen::Vector3d p = this->getKnotPos(i);
    Sophus::SO3d s = this->getKnotSO3(i);
    Eigen::Quaterniond q = s.unit_quaternion();
    outfile << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " "
            << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  outfile.close();
}

bool Trajectory::LoadTrajectoryControlPoints(std::string path) {
  std::ifstream infile;
  infile.open(path);
  std::string current_line;

  std::vector<Eigen::VectorXd> controlPoints;
  while (std::getline(infile, current_line)) {
    std::istringstream s(current_line);
    std::string field;
    std::vector<double> vec;

    while (std::getline(s, field, ' ')) {
      if (field.empty())  // Skip if empty
        continue;
      // save the data to our vector
      vec.push_back(std::atof(field.c_str()));
    }
    // Create eigen vector
    Eigen::VectorXd temp(vec.size());
    for (size_t i = 0; i < vec.size(); i++) {
      temp(i) = vec.at(i);
    }
    controlPoints.push_back(temp);
  }

  for (unsigned int i = 0; i < controlPoints.size(); i++) {
    Eigen::VectorXd temp = controlPoints.at(i);
    Eigen::Vector3d p = temp.head<3>();
    Eigen::Quaterniond q(temp(6), temp(3), temp(4), temp(5));
    Sophus::SE3d se3_knot(q, p);
    this->knots_push_back(se3_knot);
  }

  return true;
}

void Trajectory::TrajectoryToTUMTxt(std::string file_path,
                                    double relative_start_time,
                                    double relative_end_time, int iteration_num,
                                    double dt) const {
  std::string file_name = "/trajectory-lidar-" +
                          std::to_string(relative_start_time) + "-" +
                          std::to_string(relative_end_time) + "-iter" +
                          std::to_string(iteration_num) + ".txt";
  std::string traj_path = file_path + file_name;

  TrajectoryToTUMTxt2(traj_path, relative_start_time, relative_end_time, dt);
}

void Trajectory::TrajectoryToTUMTxt2(std::string traj_path,
                                     double relative_start_time,
                                     double relative_end_time,
                                     double dt) const {
  std::ofstream outfile;
  outfile.open(traj_path);

  double min_time = this->minTime();
  double max_time = this->maxTime();
  for (double t = min_time; t < max_time; t += dt) {
    double relative_bag_time = relative_start_time + t;

    SE3d pose = this->GetLidarPose(t);
    Eigen::Vector3d p = pose.translation();
    Eigen::Quaterniond q = pose.unit_quaternion();

    outfile.precision(9);
    outfile << relative_bag_time << " ";
    outfile.precision(5);
    outfile << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " "
            << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  outfile.close();
  std::cout << "Save trajectory at " << traj_path << std::endl;
}

}  // namespace liso
