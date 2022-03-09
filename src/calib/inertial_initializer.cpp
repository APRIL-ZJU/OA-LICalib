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

#include <calib/inertial_initializer.h>

namespace liso {

bool InertialInitializer::BuildProblem(
    std::shared_ptr<Trajectory> p_trajectory,
    const Eigen::aligned_vector<OdomData>& odom_data, Eigen::MatrixXd& A) {
  Eigen::aligned_vector<Eigen::Matrix4d> A_vec;
  for (size_t j = 1; j < odom_data.size(); j++) {
    size_t i = j - 1;
    double ti = odom_data.at(i).timestamp;
    double tj = odom_data.at(j).timestamp;

    if (tj > p_trajectory->maxTime()) break;

    // realtive rotation of IMU
    SE3d posei = p_trajectory->pose(ti);
    SE3d posej = p_trajectory->pose(tj);
    Eigen::Quaterniond qi = posei.unit_quaternion();
    Eigen::Quaterniond qj = posej.unit_quaternion();
    Eigen::Quaterniond delta_qij_imu = qi.conjugate() * qj;

    Eigen::Matrix3d R_Si_toS0 = odom_data.at(i).pose.topLeftCorner<3, 3>();
    Eigen::Matrix3d R_Sj_toS0 = odom_data.at(j).pose.topLeftCorner<3, 3>();
    Eigen::Matrix3d delta_ij_sensor = R_Si_toS0.transpose() * R_Sj_toS0;
    Eigen::Quaterniond delta_qij_sensor(delta_ij_sensor);

    Eigen::AngleAxisd R_vector1(delta_qij_sensor.toRotationMatrix());
    Eigen::AngleAxisd R_vector2(delta_qij_imu.toRotationMatrix());
    double delta_angle =
        180 / M_PI * std::fabs(R_vector1.angle() - R_vector2.angle());
    double huber = delta_angle > 1.0 ? 1.0 / delta_angle : 1.0;

    Eigen::Matrix4d lq_mat = mathutils::LeftQuatMatrix(delta_qij_sensor);
    Eigen::Matrix4d rq_mat = mathutils::RightQuatMatrix(delta_qij_imu);
    A_vec.push_back(huber * (lq_mat - rq_mat));
  }

  size_t valid_size = A_vec.size();
  if (valid_size < 15) {
    return false;
  }
  Eigen::MatrixXd A_temp(valid_size * 4, 4);
  for (size_t i = 0; i < valid_size; ++i)
    A_temp.block<4, 4>(i * 4, 0) = A_vec.at(i);

  A = A_temp;
  return true;
}

bool InertialInitializer::EstimateRotation(
    std::shared_ptr<Trajectory> p_trajectory,
    const Eigen::aligned_vector<OdomData>& odom_data) {
  Eigen::MatrixXd A;
  bool ret = BuildProblem(p_trajectory, odom_data, A);
  if (!ret) return false;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond q_ItoS_est(x);
  Eigen::Vector4d cov = svd.singularValues();

  if (cov(2) > 0.25) {
    q_ItoS_est_ = q_ItoS_est;
    rotaion_initialized_ = true;
    return true;
  } else {
    return false;
  }
}

bool InertialInitializer::SolveConstraintqyx(const Eigen::Vector4d t1,
                                             const Eigen::Vector4d t2,
                                             double& x1, double& x2) {
  double a = t1(0) * t1(1) + t1(2) * t1(3);
  double b = t1(0) * t2(1) + t1(1) * t2(0) + t1(2) * t2(3) + t1(3) * t2(2);
  double c = t2(0) * t2(1) + t2(2) * t2(3);

  if (std::fabs(a) < 1e-10) {
    x1 = x2 = -c / b;
    return true;
  }
  double delta2 = b * b - 4.0 * a * c;

  if (delta2 < 0.0) return false;

  double delta = sqrt(delta2);

  x1 = (-b + delta) / (2.0 * a);
  x2 = (-b - delta) / (2.0 * a);

  return true;
}

bool InertialInitializer::EstimateRotationRyx(
    std::shared_ptr<Trajectory> p_trajectory,
    const Eigen::aligned_vector<OdomData>& odom_data) {
  Eigen::MatrixXd A;
  bool ret = BuildProblem(p_trajectory, odom_data, A);
  if (!ret) return false;

  Eigen::MatrixXd AtA = A.transpose() * A;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      AtA, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector4d v1 = svd.matrixV().block<4, 1>(0, 2);
  Eigen::Vector4d v2 = svd.matrixV().block<4, 1>(0, 3);

  double lambda[2];
  if (!SolveConstraintqyx(v1, v2, lambda[0], lambda[1])) {
    std::cout << RED << "# ERROR: Quadratic equation cannot be solved "
              << "due to negative determinant." << RESET << std::endl;
    return false;
  }

  // choose one lambda
  Eigen::Matrix3d R_yxs[2];
  std::vector<Eigen::Vector3d> ypr(2);

  for (int i = 0; i < 2; ++i) {
    double t = lambda[i] * lambda[i] * v1.dot(v1) + 2 * lambda[i] * v1.dot(v2) +
               v2.dot(v2);

    double lambda2 = sqrt(1.0 / t);
    double lambda1 = lambda[i] * lambda2;
    std::cout << "lambda : " << lambda1 << " : " << lambda2 << std::endl;

    Eigen::Quaterniond q_yx;
    q_yx.coeffs() = lambda1 * v1 + lambda2 * v2;  // x,y,z,w

    R_yxs[i] = q_yx.toRotationMatrix();
    ypr[i] = mathutils::R2ypr(R_yxs[i]);
    std::cout << "ypr : " << ypr[i].transpose() << std::endl;
  }

  // q_yx  means yaw is zero. we choose the smaller yaw
  if (fabs(ypr[0](0)) < fabs(ypr[1](0))) {
    q_ItoS_est_ = Eigen::Quaterniond(R_yxs[0]);
  } else {
    q_ItoS_est_ = Eigen::Quaterniond(R_yxs[1]);
  }

  return true;
}
}  // namespace liso
