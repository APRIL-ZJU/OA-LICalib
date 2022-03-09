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

#ifndef SCALED_MISALIGNED_IMU_H
#define SCALED_MISALIGNED_IMU_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace liso {
struct IMUIntrinsic {
  IMUIntrinsic() {
    Ma_vec_.head<3>() = Eigen::Vector3d::Ones();
    Ma_vec_.tail<3>() = Eigen::Vector3d::Zero();

    Mw_vec_.head<3>() = Eigen::Vector3d::Ones();
    Mw_vec_.tail<3>() = Eigen::Vector3d::Zero();

    Aw_vec_ = Eigen::Matrix<double, 9, 1>::Zero();

    q_WtoA_ = Eigen::Quaterniond::Identity();
  }

  template <typename T>
  static void GetCalibrated(const Eigen::Matrix<T, 3, 3>& Mw,
                            const Eigen::Matrix<T, 3, 3>& Aw,
                            const Eigen::Matrix<T, 3, 3>& Ma,
                            const Eigen::Quaternion<T>& q_WtoA,
                            Eigen::Matrix<T, 3, 1>& w_b,
                            Eigen::Matrix<T, 3, 1>& a_b) {
    w_b = (Mw * (q_WtoA * w_b) + Aw * a_b).eval();
    a_b = (Ma * a_b).eval();
  }

  double* GetMwVector() { return Mw_vec_.data(); }
  double* GetMaVector() { return Ma_vec_.data(); }

  double* GetAwVector() { return Aw_vec_.data(); }

  double* GetQWtoAVector() { return q_WtoA_.coeffs().data(); }

  void ShowIMUParam() const {
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Sw," << Mw_vec_[0] << "," << Mw_vec_[1] << "," << Mw_vec_[2]
              << std::endl;
    std::cout << "Mw," << Mw_vec_[3] << "," << Mw_vec_[4] << "," << Mw_vec_[5]
              << std::endl;
    std::cout << "Sa," << Ma_vec_[0] << "," << Ma_vec_[1] << "," << Ma_vec_[2]
              << std::endl;
    std::cout << "Ma," << Ma_vec_[3] << "," << Ma_vec_[4] << "," << Ma_vec_[5]
              << std::endl;
    std::cout << "Aw," << Aw_vec_[0];
    for (int i = 1; i < 9; ++i) std::cout << "," << Aw_vec_[i];
    std::cout << std::endl;

    std::cout.unsetf(std::ios::fixed);

    Eigen::Vector3d euler_WtoA =
        (q_WtoA_.toRotationMatrix().eulerAngles(0, 1, 2)) * 180 / M_PI;
    std::cout << "euler_WtoA: " << euler_WtoA.transpose() << std::endl;

    Eigen::Vector3d euler_AtoW =
        (q_WtoA_.inverse().toRotationMatrix().eulerAngles(0, 1, 2)) * 180 /
        M_PI;
    std::cout << "euler_AtoW: " << euler_AtoW.transpose() << std::endl;
  }

 public:
  // head<3> scale; tail<3> misaligned.
  Eigen::Matrix<double, 6, 1> Mw_vec_;
  // head<3> scale; tail<3> misaligned.
  Eigen::Matrix<double, 6, 1> Ma_vec_;
  Eigen::Matrix<double, 9, 1> Aw_vec_;
  Eigen::Quaterniond q_WtoA_;
};

}  // namespace liso

#endif
