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

#ifndef IMU_FACTOR_H
#define IMU_FACTOR_H

#include <basalt/spline/ceres_spline_helper.h>
#include <basalt/spline/spline_segment.h>
#include <ceres/ceres.h>
#include <sensor_data/imu_data.h>
#include <sophus/so3.hpp>

namespace liso {
using namespace basalt;

namespace gravity_factor {

template <typename T>
Eigen::Matrix<T, 3, 1> refined_gravity(
    Eigen::Map<const Eigen::Matrix<T, 2, 1>>& g_param) {
  T cr = ceres::cos(g_param[0]);
  T sr = ceres::sin(g_param[0]);
  T cp = ceres::cos(g_param[1]);
  T sp = ceres::sin(g_param[1]);
  return Eigen::Matrix<T, 3, 1>(-sp * cr * T(GRAVITY_NORM),
                                sr * T(GRAVITY_NORM),
                                -cr * cp * T(GRAVITY_NORM));
}
}  // namespace gravity_factor

class GyroscopeWithConstantBiasFactor : public CeresSplineHelper<SplineOrder> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroscopeWithConstantBiasFactor(const IMUData& imu_data,
                                  const SplineMeta<SplineOrder>& spline_meta,
                                  double weight)
      : imu_data_(imu_data), spline_meta_(spline_meta), weight_(weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Tangent = typename Sophus::SO3<T>::Tangent;

    Eigen::Map<Tangent> residuals(sResiduals);

    size_t R_offset;  // should be zero if not estimate time offset
    double u;
    spline_meta_.ComputeSplineIndex(imu_data_.timestamp, R_offset, u);

    Tangent rot_vel;
    CeresSplineHelper<SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
        sKnots + R_offset, u, inv_dt_, nullptr, &rot_vel, nullptr);

    size_t Kont_offset = spline_meta_.NumParameters();
    Eigen::Map<Tangent const> const bias(sKnots[Kont_offset]);

    residuals = rot_vel - imu_data_.gyro.template cast<T>() + bias;
    residuals = T(weight_) * residuals;

    return true;
  }

 private:
  IMUData imu_data_;
  SplineMeta<SplineOrder> spline_meta_;
  double weight_;
  double inv_dt_;
};

class GyroAcceWithConstantBiasFactor : public CeresSplineHelper<SplineOrder> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroAcceWithConstantBiasFactor(const IMUData& imu_data,
                                 const SplineMeta<SplineOrder>& spline_meta,
                                 double gyro_weight, double acce_weight)
      : imu_data_(imu_data),
        spline_meta_(spline_meta),
        gyro_weight_(gyro_weight),
        acce_weight_(acce_weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec2T = Eigen::Matrix<T, 2, 1>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec6T = Eigen::Matrix<T, 6, 1>;
    using Vec9T = Eigen::Matrix<T, 9, 1>;
    using Mat3T = Eigen::Matrix<T, 3, 3>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    Eigen::Map<Vec6T> residuals(sResiduals);

    size_t R_offset;  // should be zero if not estimate time offset
    size_t P_offset;
    double u;
    spline_meta_.ComputeSplineIndex(imu_data_.timestamp, R_offset, u);
    P_offset = R_offset + spline_meta_.NumParameters();

    SO3T R_w_i;
    Tangent rot_vel;
    CeresSplineHelper<SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
        sKnots + R_offset, u, inv_dt_, &R_w_i, &rot_vel);

    Vec3T accel_w;
    CeresSplineHelper<SplineOrder>::template evaluate<T, 3, 2>(
        sKnots + P_offset, u, inv_dt_, &accel_w);

    size_t Kont_offset = 2 * spline_meta_.NumParameters();
    Eigen::Map<const Vec3T> gyro_bias(sKnots[Kont_offset]);
    Eigen::Map<const Vec3T> acce_bias(sKnots[Kont_offset + 1]);
    Eigen::Map<const Vec2T> g_refine(sKnots[Kont_offset + 2]);

    auto Mw_vec = sKnots[Kont_offset + 3];
    auto Ma_vec = sKnots[Kont_offset + 4];
    auto Aw_vec = sKnots[Kont_offset + 5];
    auto q_WtoA_vec = sKnots[Kont_offset + 6];
    Eigen::Map<SO3T const> const S_WtoA(q_WtoA_vec);

    Mat3T Mw = Mat3T::Zero();
    Mat3T Ma = Mat3T::Zero();
    Mat3T Aw = Mat3T::Zero();

    Mw.diagonal() = Eigen::Map<const Vec3T>(Mw_vec, 3);
    Mw(0, 1) = *(Mw_vec + 3);
    Mw(0, 2) = *(Mw_vec + 4);
    Mw(1, 2) = *(Mw_vec + 5);

    Ma.diagonal() = Eigen::Map<const Vec3T>(Ma_vec, 3);
    Ma(0, 1) = *(Ma_vec + 3);
    Ma(0, 2) = *(Ma_vec + 4);
    Ma(1, 2) = *(Ma_vec + 5);

    Aw.col(0) = Eigen::Map<const Vec3T>(Aw_vec, 3);
    Aw.col(1) = Eigen::Map<const Vec3T>(Aw_vec + 3, 3);
    Aw.col(2) = Eigen::Map<const Vec3T>(Aw_vec + 6, 3);

    Vec3T gravity = gravity_factor::refined_gravity(g_refine);

    Vec3T a_b = R_w_i.inverse() * (accel_w + gravity);
    Vec3T w_b = (Mw * (S_WtoA * rot_vel) + Aw * a_b).eval();
    a_b = (Ma * a_b).eval();

    Vec3T gyro_residuals = w_b - imu_data_.gyro.template cast<T>() + gyro_bias;
    Vec3T acce_residuals = a_b - imu_data_.accel.template cast<T>() + acce_bias;

    residuals.template block<3, 1>(0, 0) = T(gyro_weight_) * gyro_residuals;
    residuals.template block<3, 1>(3, 0) = T(acce_weight_) * acce_residuals;

    return true;
  }

 private:
  IMUData imu_data_;
  SplineMeta<SplineOrder> spline_meta_;
  double gyro_weight_;
  double acce_weight_;
  double inv_dt_;
};

class IMUPoseFactor : public CeresSplineHelper<SplineOrder> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUPoseFactor(const PoseData& pose_data,
                const SplineMeta<SplineOrder>& spline_meta, double pos_weight,
                double rot_weight)
      : pose_data_(pose_data),
        spline_meta_(spline_meta),
        pos_weight_(pos_weight),
        rot_weight_(rot_weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec6T = Eigen::Matrix<T, 6, 1>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    size_t R_offset;  // should be zero if not estimate time offset
    size_t P_offset;
    double u;
    spline_meta_.ComputeSplineIndex(pose_data_.timestamp, R_offset, u);
    P_offset = R_offset + spline_meta_.NumParameters();

    SO3T R_IkToG;
    CeresSplineHelper<SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
        sKnots + R_offset, u, inv_dt_, &R_IkToG);

    Vec3T p_IkinG;
    CeresSplineHelper<SplineOrder>::template evaluate<T, 3, 0>(
        sKnots + P_offset, u, inv_dt_, &p_IkinG);

    Eigen::Map<Vec6T> residuals(sResiduals);
    residuals.template block<3, 1>(0, 0) =
        T(rot_weight_) * (R_IkToG * pose_data_.orientation.inverse()).log();

    residuals.template block<3, 1>(3, 0) =
        T(pos_weight_) * (p_IkinG - pose_data_.position);
    return true;
  }

 private:
  PoseData pose_data_;
  SplineMeta<SplineOrder> spline_meta_;
  double pos_weight_;
  double rot_weight_;
  double inv_dt_;
};

class IMUPositionFactor : public CeresSplineHelper<SplineOrder> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUPositionFactor(const PoseData& pose_data,
                    const SplineMeta<SplineOrder>& spline_meta,
                    double pos_weight)
      : pose_data_(pose_data),
        spline_meta_(spline_meta),
        pos_weight_(pos_weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec6T = Eigen::Matrix<T, 6, 1>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    size_t P_offset;
    double u;
    spline_meta_.ComputeSplineIndex(pose_data_.timestamp, P_offset, u);

    Vec3T p_IkinG;
    CeresSplineHelper<SplineOrder>::template evaluate<T, 3, 0>(
        sKnots + P_offset, u, 1, &p_IkinG);

    Eigen::Map<Vec3T> residuals(sResiduals);

    residuals = T(pos_weight_) * (p_IkinG - pose_data_.position);
    return true;
  }

 private:
  PoseData pose_data_;
  SplineMeta<SplineOrder> spline_meta_;
  double pos_weight_;
  double inv_dt_;
};

class IMUOrientationFactor : public CeresSplineHelper<SplineOrder> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUOrientationFactor(const PoseData& pose_data,
                       const SplineMeta<SplineOrder>& spline_meta,
                       double rot_weight)
      : pose_data_(pose_data),
        spline_meta_(spline_meta),
        rot_weight_(rot_weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec6T = Eigen::Matrix<T, 6, 1>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    size_t R_offset;  // should be zero if not estimate time offset
    double u;
    spline_meta_.ComputeSplineIndex(pose_data_.timestamp, R_offset, u);

    SO3T R_IkToG;
    CeresSplineHelper<SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
        sKnots, u, 1, &R_IkToG);

    Eigen::Map<Tangent> residuals(sResiduals);

    residuals =
        T(rot_weight_) * ((R_IkToG * pose_data_.orientation.inverse()).log());
    return true;
  }

 private:
  PoseData pose_data_;
  SplineMeta<SplineOrder> spline_meta_;
  double rot_weight_;
  double inv_dt_;
};

}  // namespace liso

#endif
