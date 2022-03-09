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

#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <fstream>
#include <memory>

#include <sensor_data/cloud_type.h>
#include <sensor_data/imu_data.h>
#include <sensor_data/scaled_misaligned_imu.h>
#include <utils/yaml_utils.h>
#include <utils/eigen_utils.hpp>

#include <factor/auto_diff/imu_factor.h>
#include <factor/auto_diff/lidar_feature_factor.h>

// the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

namespace liso {

struct SegmentCalibParam {
  SegmentCalibParam()
      : g_refine(Eigen::Vector2d(0, 0)),
        gyro_bias(Eigen::Vector3d(0, 0, 0)),
        acce_bias(Eigen::Vector3d(0, 0, 0)),
        gravity(Eigen::Vector3d(0, 0, GRAVITY_NORM)),
        time_offset(0) {}

  Eigen::Vector2d g_refine;

  Eigen::Vector3d gyro_bias;

  Eigen::Vector3d acce_bias;

  Eigen::Vector3d gravity;

  double time_offset;
};

struct NDTLocatorParam {
  NDTLocatorParam()
      : ndt_prior_map_path(""),
        locator_init_pose(Eigen::Matrix4d::Identity()) {}

  std::string ndt_prior_map_path;
  Eigen::Matrix4d locator_init_pose;
};

struct LiDAROdomParam {
  double ndt_resolution = 0.5;
  double ndt_key_frame_downsample = 0.1;
  double map_downsample_size = 0.5;

  double scan4map_time = 10;

  LiDAROdomParam() {}

  LiDAROdomParam(const YAML::Node& node) {
    ndt_resolution = node["ndtResolution"].as<double>();
    ndt_key_frame_downsample = node["ndt_key_frame_downsample"].as<double>();
    map_downsample_size = node["map_downsample_size"].as<double>();

    scan4map_time = node["scan4map"].as<double>();
  }
};

struct CalibWeights {
  CalibWeights() {}

  CalibWeights(const YAML::Node& config_node) {
    /// estimate weight param
    opt_gyro_weight = config_node["gyro_weight"].as<double>();   // 28.0;
    opt_acce_weight = config_node["accel_weight"].as<double>();  // 18.5;

    if (config_node["lidar_weight"])
      opt_lidar_weight = config_node["lidar_weight"].as<double>();  // 10.0;
  }

  /// weight
  double opt_gyro_weight;

  double opt_acce_weight;

  double opt_lidar_weight;
};

struct CalibOptions {
  CalibOptions() {}

  CalibOptions(const YAML::Node& node) {
    is_plane_motion = node["plane_motion"].as<bool>();

    opt_time_offset = node["opt_timeoffset"].as<bool>();
    time_offset_padding = node["timeoffset_padding"].as<double>();

    lock_opt_first_accel_bias = node["lock_accel_bias"].as<bool>();

    opt_lidar_intrinsic = node["opt_lidar_intrinsic"].as<bool>();
    opt_IMU_intrinsic = node["opt_IMU_intrinsic"].as<bool>();

    // apply_lidar_intrinstic_to_scan = false;

    std::cout << YELLOW << "time_offset_padding set as: " << time_offset_padding
              << RESET << std::endl;
  }

  bool is_plane_motion = false;

  bool opt_time_offset = false;
  double time_offset_padding = 0;

  bool lock_opt_first_accel_bias = false;

  bool opt_lidar_intrinsic = false;
  bool opt_IMU_intrinsic = false;

  bool apply_lidar_intrinstic_to_scan = false;
};

class CalibParamManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CalibParamManager> Ptr;

  CalibParamManager(const YAML::Node& config_node)
      : p_LinI(Eigen::Vector3d(0, 0, 0)),
        q_LtoI(Eigen::Quaterniond::Identity()) {
    lo_param = LiDAROdomParam(config_node);
    calib_weights = CalibWeights(config_node);
    calib_option = CalibOptions(config_node);

    int segment_num = config_node["segment_num"].as<int>();

    const YAML::Node& segment_node = config_node["selected_segment"];
    for (int i = 0; i < segment_num; i++) {
      std::pair<double, double> segment_t;
      segment_t.first = segment_node[i]["start_time"].as<double>();
      segment_t.second = segment_node[i]["end_time"].as<double>();
      segment_timestamp.push_back(segment_t);

      // ndt locator
      if (segment_node[i]["ndt_prior_map"]) {
        std::string ndt_prior_map_path =
            segment_node[i]["ndt_prior_map"].as<std::string>();

        std::vector<double> rpyxyz_vec;
        yaml::GetValues<double>(segment_node[i], "locator_init_rpyxyz_pose", 6,
                                rpyxyz_vec);
        for (int i = 0; i < 3; ++i) rpyxyz_vec[i] *= (M_PI / 180.);

        tf2::Quaternion quat;
        quat.setRPY(rpyxyz_vec[0], rpyxyz_vec[1], rpyxyz_vec[2]);
        Eigen::Quaterniond eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());

        Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
        init_pose.block<3, 3>(0, 0) = eigen_quat.toRotationMatrix();
        init_pose.block<3, 1>(0, 3) =
            Eigen::Vector3d(rpyxyz_vec[3], rpyxyz_vec[4], rpyxyz_vec[5]);

        locator_segment_param.emplace_back();
        locator_segment_param.back().ndt_prior_map_path = ndt_prior_map_path;
        locator_segment_param.back().locator_init_pose = init_pose;
      }
    }
    segment_param.resize(segment_num);

    /// extrinsic translation
    std::vector<double> params_vec;
    yaml::GetValues<double>(config_node["extrinsic"], "Trans", 3, params_vec);
    p_LinI << params_vec[0], params_vec[1], params_vec[2];
    /// extrinsic rotation
    params_vec.clear();
    yaml::GetValues<double>(config_node["extrinsic"], "Rot", 9, params_vec);
    Eigen::Matrix3d rot;
    rot << params_vec[0], params_vec[1], params_vec[2], params_vec[3],
        params_vec[4], params_vec[5], params_vec[6], params_vec[7],
        params_vec[8];

    if (config_node["extrinsic"]["Trans_prior"]) {
      yaml::GetValues<double>(config_node["extrinsic"], "Trans_prior", 3,
                              params_vec);
      p_LinI_prior << params_vec[0], params_vec[1], params_vec[2];
    } else {
      p_LinI_prior = p_LinI;
    }

    q_LtoI = Eigen::Quaterniond(rot);
    q_LtoI.normalized();
    so3_LtoI = SO3d(q_LtoI);
    se3_LtoI = SE3d(so3_LtoI, p_LinI);

    if (config_node["vlp16_ring_case"]) {
      int ring_case = config_node["vlp16_ring_case"].as<int>();
      lidar_intrinsic.SetRingCase(ring_case);
    }
  }

  void ResetTimeOffset() {
    for (size_t i = 0; i < segment_param.size(); i++) {
      segment_param[i].time_offset = 0.0;
    }
  }

  // Update after Ceres optimization
  void UpdateExtrinicParam() {
    q_LtoI = so3_LtoI.unit_quaternion();
    se3_LtoI = SE3d(so3_LtoI, p_LinI);
  }

  void UpdateGravity(Eigen::Vector3d gravity_in, int segment_id = 0) {
    assert(segment_id < segment_param.size() &&
           "[UpdateGravity] segment_id too big");
    segment_param.at(segment_id).gravity = gravity_in;

    gravity_in = (gravity_in / GRAVITY_NORM).eval();
    double cr = std::sqrt(gravity_in[0] * gravity_in[0] +
                          gravity_in[2] * gravity_in[2]);
    segment_param.at(segment_id).g_refine[0] = std::acos(cr);
    segment_param.at(segment_id).g_refine[1] = std::acos(-gravity_in[2] / cr);
  }

  void UpdateGravity() {
    for (size_t i = 0; i < segment_param.size(); i++) {
      Eigen::Map<const Eigen::Matrix<double, 2, 1>> g_param(
          segment_param.at(i).g_refine.data());
      segment_param.at(i).gravity =
          gravity_factor::refined_gravity<double>(g_param);
    }
  }

  // ========================= Visualization ========================= //
  static void friendly_output(const Eigen::VectorXd& data, const int dim,
                              const std::string description,
                              int precision = 2) {
    std::cout << std::fixed << std::setprecision(precision);
    std::cout << std::setw(15) << description;
    for (size_t i = 0; i < dim; i++) {
      std::cout << ", " << data[i];
    }
    std::cout << std::endl;
    std::cout.unsetf(std::ios::fixed);
  }

  static void friendly_output(double data, const std::string description,
                              int precision = 2) {
    std::cout << std::fixed << std::setprecision(precision);
    std::cout << std::setw(15) << description;
    std::cout << ", " << data << std::endl;
    std::cout.unsetf(std::ios::fixed);
  }

  void showStates() const {
    Eigen::Vector3d euler_LtoI = q_LtoI.toRotationMatrix().eulerAngles(0, 1, 2);
    euler_LtoI = euler_LtoI * 180 / M_PI;
    Eigen::Quaterniond q_ItoL = q_LtoI.inverse();

    Eigen::Vector3d p_IinL = q_ItoL * (-p_LinI);
    Eigen::Vector3d euler_ItoL = q_ItoL.toRotationMatrix().eulerAngles(0, 1, 2);
    euler_ItoL = euler_ItoL * 180 / M_PI;

    friendly_output(p_LinI, 3, "P_LinI", 3);
    friendly_output(euler_LtoI, 3, "euler_LtoI", 2);
    friendly_output(p_IinL, 3, "p_IinL", 3);
    friendly_output(euler_ItoL, 3, "euler_ItoL", 2);

    for (size_t i = 0; i < segment_param.size(); i++) {
      std::cout << "=========================" << std::endl;
      std::cout << "segment id  : " << i << std::endl;
      friendly_output(segment_param[i].time_offset, "time offset", 4);
      friendly_output(segment_param[i].g_refine * 180 / M_PI, 2, "g_refine", 2);
      friendly_output(segment_param[i].gravity, 3, "gravity", 3);
      friendly_output(segment_param[i].acce_bias, 3, "accel bias", 4);
      friendly_output(segment_param[i].gyro_bias, 3, "gyro bias", 4);
    }

    if (calib_option.opt_lidar_intrinsic) lidar_intrinsic.ShowLaserParam();

    if (calib_option.opt_IMU_intrinsic) {
      imu_intrinsic.ShowIMUParam();
    }
  }

  std::string ParamString() const {
    Eigen::Quaterniond q_ItoL = q_LtoI.inverse();
    Eigen::Vector3d p_IinL = q_ItoL * (-p_LinI);

    std::stringstream ss;

    // for(size_t i = 0; i < segment_param.size(); i++)
    {
      auto& v = segment_param[0];
      std::pair<double, double> segment_t = segment_timestamp.at(0);

      ss << segment_t.first << "," << segment_t.second << "," << p_IinL(0)
         << "," << p_IinL(1) << "," << p_IinL(2) << "," << q_ItoL.x() << ","
         << q_ItoL.y() << "," << q_ItoL.z() << "," << q_ItoL.w() << ","
         << v.time_offset << "," << v.g_refine(0) << "," << v.gravity(1) << ","
         << v.gravity(0) << "," << v.gravity(1) << "," << v.gravity(2) << ","
         << v.gyro_bias(0) << "," << v.gyro_bias(1) << "," << v.gyro_bias(2)
         << "," << v.acce_bias(0) << "," << v.acce_bias(1) << ","
         << v.acce_bias(2);
    }

    std::string param_info;
    ss >> param_info;
    return param_info;
  }

 public:
  LiDAROdomParam lo_param;

  CalibWeights calib_weights;

  CalibOptions calib_option;

  Eigen::Vector3d p_LinI;
  SO3d so3_LtoI;
  Eigen::Quaterniond q_LtoI;
  SE3d se3_LtoI;

  Eigen::Vector3d p_LinI_prior;

  std::vector<SegmentCalibParam> segment_param;
  std::vector<std::pair<double, double>> segment_timestamp;
  // ndt locator initial pose
  std::vector<NDTLocatorParam> locator_segment_param;

  LiDARIntrinsic lidar_intrinsic;
  IMUIntrinsic imu_intrinsic;
};

}  // namespace liso

#endif
