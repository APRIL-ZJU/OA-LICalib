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

#pragma once

#include <basalt/spline/ceres_spline_helper.h>
#include <basalt/spline/spline_segment.h>
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

namespace liso {
using namespace basalt;

class PlaneConstraintFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlaneConstraintFactor(SO3d R_plane,
                        const SplineMeta<SplineOrder>& spline_meta,
                        Eigen::Matrix3d pos_weight, Eigen::Matrix3d rot_weight)
      : R_plane_(R_plane),
        spline_meta_(spline_meta),
        pos_weight_(pos_weight),
        rot_weight_(rot_weight) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Group = typename Sophus::SO3<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Mat3T = Eigen::Matrix<T, 3, 3>;

    Mat3T pos_weight = pos_weight_.cast<T>();
    Mat3T rot_weight = rot_weight_.cast<T>();

    Vec3T e_3(T(0), T(0), T(1));

    size_t P_offset = spline_meta_.NumParameters();

    Eigen::Map<Group const> const plane_rotation(sKnots[P_offset * 2]);

    // R_plane ==> transform point from global frame to plane frame
    for (size_t i = 0; i < spline_meta_.NumParameters(); ++i) {
      Eigen::Map<Group const> const R_i(sKnots[i]);
      Vec3T error = rot_weight * (plane_rotation * R_i * e_3 - e_3);
      Eigen::Map<Vec3T> residuals(sResiduals + 3 * i);
      residuals = error;

      Eigen::Map<Vec3T const> const p_i(sKnots[i + P_offset]);
      T p_error = pos_weight(0, 0) * (plane_rotation * p_i).dot(e_3);
      sResiduals[3 * P_offset + i] = p_error;
    }
    return true;
  }

 private:
  SO3d R_plane_;
  SplineMeta<SplineOrder> spline_meta_;
  Eigen::Matrix3d pos_weight_;
  Eigen::Matrix3d rot_weight_;
};

/// Reference
/// https://github.com/ethz-asl/kalibr/blob/master/aslam_nonparametric_estimation/bsplines/src/BSpline.cpp#L1550
/// https://doi.org/10.1177/0278364915585860 (Section 4.6)
template <int _N, int DERIV>
class QuadraticIntegralFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadraticIntegralFactor(std::vector<double> times,
                          const SplineMeta<SplineOrder>& spline_meta,
                          const Eigen::Matrix3d& weight)
      : times_(times), spline_meta_(spline_meta), W_(weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;

    Eigen::Matrix<double, _N, _N> Vi = Eigen::MatrixXd::Zero(_N, _N);
    Eigen::Matrix<double, _N, _N> Di = Eigen::MatrixXd::Zero(_N, _N);
    for (size_t h = 0; h < _N; ++h) {
      for (size_t w = 0; w < _N; ++w) {
        Vi(h, w) = 1.0 / double(w + h + 1);
      }
    }
    Vi *= spline_meta_.segments.begin()->dt;

    for (int i = 0; i < _N - 1; i++) {
      Di(i, i + 1) = (i + 1.0) * inv_dt_;
    }

    for (int i = 0; i < DERIV; i++) {
      Vi = (Di.transpose() * Vi * Di).eval();
    }

    Eigen::Matrix<double, _N, _N> M =
        CeresSplineHelper<_N>::blending_matrix_.transpose();
    Qi_ = M.transpose() * Vi * M;

    auto svd = Qi_.jacobiSvd(Eigen::ComputeFullU);
    Ri_ = (svd.matrixU() *
           svd.singularValues().array().sqrt().matrix().asDiagonal())
              .transpose();

    //    std::cout << "DERIV: " << DERIV << std::endl;
    //    std::cout << "dt, inv_dt: " << spline_meta_.segments.begin()->dt
    //              << "|" << inv_dt_ << std::endl;
    //    std::cout << "M: \n" << M << std::endl;
    //    std::cout << "Di: \n" << Di << std::endl;
    //    std::cout << "Vi: \n" << Vi << std::endl;
    //    std::cout << "Qi_: \n" << Qi_ << std::endl;
  }

  template <class T>
  void MatrixMultiplication(T const* const* Knots, T* Residuals) const {
    using VecN = Eigen::Matrix<T, _N, 1>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vec3T> residuals(Residuals);

    Eigen::Matrix<T, _N, _N> Qi = Qi_.template cast<T>();

    // Residuals = Ct*Q*C = (1,_N) * (_N,_N) * (_N,1)
    for (int k = 0; k < 3; k++) {
      VecN CtQ = VecN::Zero();
      for (int i = 0; i < _N; i++) {
        for (int j = 0; j < _N; j++) {
          CtQ(i) += Knots[j][k] * Qi(j, i);
        }
      }
      residuals(k) = T(0);
      for (int i = 0; i < _N; i++) {
        residuals(k) += CtQ(i) * Knots[i][k];
      }
    }
    residuals = W_.template cast<T>() * residuals;
  }

  template <class T>
  void Rc(T const* const* Knots, T* Residuals) const {
    using Vec3N = Eigen::Matrix<T, 3 * _N, 1>;

    Eigen::Map<Vec3N> residuals(Residuals);

    Eigen::Matrix<T, _N, _N> Ri = Ri_.template cast<T>();

    // Residuals = R*c = (_N,_N) * (_N,1)
    Vec3N rc = Vec3N::Zero();
    for (int k = 0; k < 3; k++) {
      for (int i = 0; i < _N; i++) {
        for (int j = 0; j < _N; j++) {
          rc(i * 3 + k) += Ri_(i, j) * Knots[j][k];
        }
      }
    }
    residuals = T(W_(0, 0)) * rc;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using VecN = Eigen::Matrix<T, _N, 1>;
    using MatN = Eigen::Matrix<T, _N, _N>;

    size_t cnt = 0;
    for (auto const& t : times_) {
      size_t P_offset;  // should be zero if not estimate time offset
      double u;
      if (!spline_meta_.ComputeSplineIndex(t, P_offset, u)) {
        std::cout << t << "; P_offset " << P_offset << "; "
                  << spline_meta_.segments.front().MinTime() << "; "
                  << spline_meta_.NumParameters() << std::endl;
        std::cerr
            << "[QuadraticIntegralFactor::operator()] : "
            << " maybe std::floor function suffers from precision problem\n";
      }

#if false
      MatrixMultiplication<T>(sKnots+P_offset, sResiduals+cnt);
      cnt += 3;
#else
      Rc<T>(sKnots + P_offset, sResiduals + cnt);
      cnt += 3 * _N;
#endif
    }
    return true;
  }

 private:
  std::vector<double> times_;
  SplineMeta<_N> spline_meta_;
  const Eigen::Matrix3d W_;
  double inv_dt_;

  Eigen::Matrix<double, _N, _N> Qi_;
  Eigen::Matrix<double, _N, _N> Ri_;
};

class AngularVelocityConvexHullFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AngularVelocityConvexHullFactor(const SplineMeta<SplineOrder>& spline_meta,
                                  double weight)
      : spline_meta_(spline_meta), weight_(weight) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* residuals) const {
    using Tangent = typename Sophus::SO3<T>::Tangent;
    using SO3 = Sophus::SO3<T>;

    T dt = T(spline_meta_.segments.front().dt);
    size_t idx = 0;
    size_t res_idx = 0;
    for (SplineSegmentMeta<SplineOrder> const& seg : spline_meta_.segments) {
      for (size_t i = 0; i < seg.NumParameters() - 1; ++i) {
        Eigen::Map<SO3 const> const p0(sKnots[idx + i]);
        Eigen::Map<SO3 const> const p1(sKnots[idx + i + 1]);

        Tangent r01 = (p0.inverse() * p1).log();
        residuals[res_idx + i] = T(weight_ * dt) * r01.norm();
      }
      idx += seg.NumParameters();
      res_idx += seg.NumParameters() - 1;
    }

    return true;
  }

 private:
  SplineMeta<SplineOrder> spline_meta_;
  double weight_;
};

}  // namespace liso
