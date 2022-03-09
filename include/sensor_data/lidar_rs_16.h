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

#ifndef _ROBOSENSE_CORRECTION_HPP_
#define _ROBOSENSE_CORRECTION_HPP_

#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>

#include <angles/angles.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include <sensor_data/lidar_feature.h>

#define RS_TO_RADS(x) ((x) * (M_PI) / 180)

namespace liso {

/**
 * @brief The LidarCorrection class
 * robosense
 */
class RobosenseCorrection {
  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<RobosenseCorrection> Ptr;

  enum ModelType { RS_16, RS_32 };

  RobosenseCorrection(ModelType model_type) {
    m_modelType = model_type;
    if (m_modelType == ModelType::RS_16) {
      numOfLasers = 16;
      Rx_ = 0.03825;
      Ry_ = -0.01088;
      Rz_ = 0;
    } else if (m_modelType == ModelType::RS_32) {
      numOfLasers = 32;
      Rx_ = 0.03997;
      Ry_ = -0.01087;
      Rz_ = 0;
    }

    tempPacketNum = 0;
    temper = 31.0;
    dis_resolution_mode = 0;

    is_init_angle_ = false;
    is_init_curve_ = false;
    is_init_top_fw_ = false;

    intensity_mode_ = 1;

    // lookup table init
    this->cos_lookup_table_.resize(36000);
    this->sin_lookup_table_.resize(36000);
    for (unsigned int i = 0; i < 36000; i++) {
      double rad = RS_TO_RADS(i / 100.0f);

      this->cos_lookup_table_[i] = std::cos(rad);
      this->sin_lookup_table_[i] = std::sin(rad);
    }

    return_mode_ = 1;

    for (unsigned int w = 0; w < 2016; w++) {
      for (unsigned int h = 0; h < 16; h++) {
        mRS16TimeBlock[w][h] =
            h * 3.0 * 1e-6 + w * 50.0 * 1e-6;  /// rs_lidar 16*2016
      }
    }
  }

  void unpack_scan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg,
                   LiDARFeature& output) {
    if (is_init_angle_ && is_init_top_fw_) {
      if (m_modelType == ModelType::RS_16) {
        unpack_rs16(scanMsg, output);
      } else if (m_modelType == ModelType::RS_32) {
        //           unpack_rs32(scanMsg, output);
        std::cout << "coming soon " << std::endl;
      }
    } else {
      std::cout << "Unpack Error! There is no Difop message!" << std::endl;
    }
  }

  void unpack_rs16(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg,
                   LiDARFeature& output) {
    output.Clear();
    output.timestamp = scanMsg->header.stamp.toSec();
    output.full_features->height = 16;
    output.full_features->width = 24 * (int)scanMsg->packets.size();
    output.full_features->is_dense = false;
    output.full_features->resize(output.full_features->height *
                                 output.full_features->width);

    /// raw_data
    output.raw_data->height = 16;
    output.raw_data->width = 24 * (int)scanMsg->packets.size();
    output.raw_data->is_dense = false;
    output.raw_data->resize(output.raw_data->height * output.raw_data->width);

    int block_counter = 0;
    double scan_timestamp = scanMsg->header.stamp.toSec();

    float deg2rad_resolution = 1.0 / 100.0 / 180.0 * M_PI;

    for (size_t i = 0; i < scanMsg->packets.size(); i++) {
      float azimuth;  // 0.01 dgree
      float intensity;
      float azimuth_diff;
      float azimuth_corrected_f;
      int azimuth_corrected;

      const raw_packet_t* raw =
          (const raw_packet_t*)&scanMsg->packets[i].data[42];
      if (tempPacketNum < 75 && tempPacketNum > 0) {
        tempPacketNum++;
      } else {
        temper = computeTemperature(scanMsg->packets[i].data[38],
                                    scanMsg->packets[i].data[39]);
        tempPacketNum = 1;
      }

      for (int block = 0; block < BLOCKS_PER_PACKET; block++, block_counter++) {
        if (UPPER_BANK != raw->blocks[block].header) {
          ROS_INFO_STREAM_THROTTLE(180, "skipping RSLIDAR DIFOP packet");
          break;
        }

        azimuth = (float)(256 * raw->blocks[block].rotation_1 +
                          raw->blocks[block].rotation_2);

        int azi1, azi2;
        if (block < (BLOCKS_PER_PACKET - 1)) {
          azi1 = 256 * raw->blocks[block + 1].rotation_1 +
                 raw->blocks[block + 1].rotation_2;
          azi2 = 256 * raw->blocks[block].rotation_1 +
                 raw->blocks[block].rotation_2;
        } else {
          azi1 = 256 * raw->blocks[block].rotation_1 +
                 raw->blocks[block].rotation_2;
          azi2 = 256 * raw->blocks[block - 1].rotation_1 +
                 raw->blocks[block - 1].rotation_2;
        }

        uint16_t diff = (36000 + azi1 - azi2) % 36000;
        if (diff > 100)  // to avoid when the lidar is set to specific FOV that
                         // cause the big difference between angle
        {
          diff = 0;
        }
        azimuth_diff = (float)(diff);

        for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++) {
          for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING;
               dsr++, k += RAW_SCAN_SIZE) {
            if (0 == return_mode_) {
              azimuth_corrected_f =
                  azimuth + (azimuth_diff * (dsr * RS16_DSR_TOFFSET)) /
                                RS16_FIRING_TOFFSET;
            } else {
              azimuth_corrected_f =
                  azimuth +
                  (azimuth_diff *
                   ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
                   RS16_BLOCK_TDURATION);
            }
            azimuth_corrected = ((int)round(azimuth_corrected_f)) %
                                36000;  // convert to integral value...

            union two_bytes tmp;
            tmp.bytes[1] = raw->blocks[block].data[k];
            tmp.bytes[0] = raw->blocks[block].data[k + 1];
            int distance = tmp.uint;

            // read intensity
            intensity = raw->blocks[block].data[k + 2];
            intensity = calibrateIntensity(intensity, dsr, distance);

            float distance2 =
                pixelToDistance(distance, dsr);  //矫正传感器安装距离误差
            if (dis_resolution_mode == 0) {
              distance2 = distance2 * DISTANCE_RESOLUTION_NEW;
            } else {
              distance2 = distance2 * DISTANCE_RESOLUTION;
            }

            //              float arg_horiz = (float)azimuth_corrected /
            //              18000.0f * M_PI; float arg_vert = VERT_ANGLE[dsr];
            int arg_horiz = (azimuth_corrected + 36000) % 36000;
            int arg_horiz_orginal = arg_horiz;
            int arg_vert = ((VERT_ANGLE[dsr]) % 36000 + 36000) % 36000;

            double point_timestamp =
                scan_timestamp + getExactTime(dsr, 2 * block_counter + firing);

            PosPoint point_xyz;
            PosPoint point_raw;
            point_xyz.timestamp = point_timestamp;
            point_raw.timestamp = point_timestamp;

            if (distance2 > DISTANCE_MAX ||
                distance2 < DISTANCE_MIN)  // invalid data
            {
              point_xyz.x = NAN;
              point_xyz.y = NAN;
              point_xyz.z = NAN;
              point_raw.x = NAN;
              point_raw.y = NAN;
              point_raw.z = NAN;
            } else {
              point_xyz.x = distance2 * this->cos_lookup_table_[arg_vert] *
                                this->cos_lookup_table_[arg_horiz] +
                            Rx_ * this->cos_lookup_table_[arg_horiz_orginal];
              point_xyz.y = -distance2 * this->cos_lookup_table_[arg_vert] *
                                this->sin_lookup_table_[arg_horiz] -
                            Rx_ * this->sin_lookup_table_[arg_horiz_orginal];
              point_xyz.z = distance2 * this->sin_lookup_table_[arg_vert] + Rz_;

              point_raw.x = dsr;
              point_raw.y = azimuth_corrected_f * deg2rad_resolution;
              point_raw.z = distance2;
            }

            output.full_features->at(2 * block_counter + firing, dsr) =
                point_xyz;
            output.raw_data->at(2 * block_counter + firing, dsr) = point_raw;
          }
        }
      }
    }
  }

  float pixelToDistance(int pixelValue, int passageway) {
    float DistanceValue;
    int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
    if (pixelValue <= g_ChannelNum[passageway][indexTemper]) {
      DistanceValue = 0.0;
    } else {
      DistanceValue =
          (float)(pixelValue - g_ChannelNum[passageway][indexTemper]);
    }
    return DistanceValue;
  }

  int estimateTemperature(float Temper) {
    int temp = (int)floor(Temper + 0.5);
    if (temp < TEMPERATURE_MIN) {
      temp = TEMPERATURE_MIN;
    } else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE) {
      temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
    }

    return temp;
  }

  float computeTemperature(unsigned char bit1, unsigned char bit2) {
    float Temp;
    float bitneg = bit2 & 128;   // 10000000
    float highbit = bit2 & 127;  // 01111111
    float lowbit = bit1 >> 3;
    if (bitneg == 128) {
      Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
    } else {
      Temp = (highbit * 32 + lowbit) * 0.0625f;
    }

    return Temp;
  }

  float calibrateIntensity(float intensity, int calIdx, int distance) {
    int algDist;
    int sDist;
    int uplimitDist;
    float realPwr;
    float refPwr;
    float tempInten;
    float distance_f;
    float endOfSection1, endOfSection2;

    int temp = estimateTemperature(temper);

    realPwr = std::max(
        (float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
    // realPwr = intensity;

    if (intensity_mode_ == 1) {
      // transform the one byte intensity value to two byte
      if ((int)realPwr < 126)
        realPwr = realPwr * 4.0f;
      else if ((int)realPwr >= 126 && (int)realPwr < 226)
        realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
      else
        realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;
    } else if (intensity_mode_ == 2) {
      // the caculation for the firmware after T6R23V8(16) and T9R23V6(32)
      if ((int)realPwr < 64)
        realPwr = realPwr;
      else if ((int)realPwr >= 64 && (int)realPwr < 176)
        realPwr = (realPwr - 64.0f) * 4.0f + 64.0f;
      else
        realPwr = (realPwr - 176.0f) * 16.0f + 512.0f;
    } else {
      // std::cout << "The intensity mode is not right" << std::endl;
    }

    int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
    uplimitDist = g_ChannelNum[calIdx][indexTemper] + 20000;
    // limit sDist
    sDist = (distance > g_ChannelNum[calIdx][indexTemper])
                ? distance
                : g_ChannelNum[calIdx][indexTemper];
    sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
    // minus the static offset (this data is For the intensity cal useage only)
    algDist = sDist - g_ChannelNum[calIdx][indexTemper];

    // calculate intensity ref curves
    float refPwr_temp = 0.0f;
    int order = 3;
    endOfSection1 = 500.0f;
    endOfSection2 = 4000.0;
    distance_f = (float)algDist;
    if (intensity_mode_ == 1) {
      if (distance_f <= endOfSection1) {
        refPwr_temp = aIntensityCal[0][calIdx] *
                          exp(aIntensityCal[1][calIdx] -
                              aIntensityCal[2][calIdx] * distance_f / 100.0f) +
                      aIntensityCal[3][calIdx];
        //   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
      } else {
        for (int i = 0; i < order; i++) {
          refPwr_temp += aIntensityCal[i + 4][calIdx] *
                         (pow(distance_f / 100.0f, order - 1 - i));
        }
        // printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
      }
    } else if (intensity_mode_ == 2) {
      if (distance_f <= endOfSection1) {
        refPwr_temp = aIntensityCal[0][calIdx] *
                          exp(aIntensityCal[1][calIdx] -
                              aIntensityCal[2][calIdx] * distance_f / 100.0f) +
                      aIntensityCal[3][calIdx];
        //   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
      } else if (distance_f > endOfSection1 && distance_f <= endOfSection2) {
        for (int i = 0; i < order; i++) {
          refPwr_temp += aIntensityCal[i + 4][calIdx] *
                         (pow(distance_f / 100.0f, order - 1 - i));
        }
        // printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
      } else {
        float refPwr_temp0 = 0.0f;
        float refPwr_temp1 = 0.0f;
        for (int i = 0; i < order; i++) {
          refPwr_temp0 += aIntensityCal[i + 4][calIdx] *
                          (pow(4000.0f / 100.0f, order - 1 - i));
          refPwr_temp1 += aIntensityCal[i + 4][calIdx] *
                          (pow(3900.0f / 100.0f, order - 1 - i));
        }
        refPwr_temp =
            0.3f * (refPwr_temp0 - refPwr_temp1) * distance_f / 100.0f +
            refPwr_temp0;
      }
    } else {
      // std::cout << "The intensity mode is not right" << std::endl;
    }

    refPwr = std::max(std::min(refPwr_temp, 500.0f), 4.0f);

    tempInten = (intensityFactor * refPwr) / realPwr;
    if (m_modelType == ModelType::RS_32) {
      tempInten = tempInten * CurvesRate[calIdx];
    }
    tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
    return tempInten;
  }

  void processDifop(const rslidar_msgs::rslidarPacket::ConstPtr& difop_msg) {
    // std::cout << "Enter difop callback!" << std::endl;
    const uint8_t* data = &difop_msg->data[0];
    bool is_support_dual_return = false;

    // check header
    if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 ||
        data[3] != 0x5a) {
      return;
    }

    if ((data[45] == 0x08 && data[46] == 0x02 && data[47] >= 0x09) ||
        (data[45] > 0x08) || (data[45] == 0x08 && data[46] > 0x02)) {
      is_support_dual_return = true;
      if (data[300] == 0x01 || data[300] == 0x02) {
        return_mode_ = data[300];
      } else {
        return_mode_ = 0;
      }
    } else {
      is_support_dual_return = false;
      return_mode_ = 1;
    }

    if (!this->is_init_top_fw_) {
      if ((data[41] == 0x00 && data[42] == 0x00 && data[43] == 0x00) ||
          (data[41] == 0xff && data[42] == 0xff && data[43] == 0xff) ||
          (data[41] == 0x55 && data[42] == 0xaa && data[43] == 0x5a) ||
          (data[41] == 0xe9 && data[42] == 0x01 && data[43] == 0x00)) {
        dis_resolution_mode = 1;  // 1cm resolution
        std::cout << "The distance resolution is 1cm" << std::endl;
      } else {
        dis_resolution_mode = 0;  // 0.5cm resolution
        std::cout << "The distance resolution is 0.5cm" << std::endl;
      }
      this->is_init_top_fw_ = true;
    }

    if (!this->is_init_curve_) {
      // check header
      if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 &&
          data[3] == 0x5a) {
        bool curve_flag = true;
        // check difop reigon has beed flashed the right data
        if ((data[50] == 0x00 || data[50] == 0xff) &&
            (data[51] == 0x00 || data[51] == 0xff) &&
            (data[52] == 0x00 || data[52] == 0xff) &&
            (data[53] == 0x00 || data[53] == 0xff)) {
          curve_flag = false;
        }
        // TODO check why rsview here no 32 laser, be more careful the new, old
        // version Init curves
        if (curve_flag) {
          unsigned char checkbit;
          int bit1, bit2;
          for (int loopn = 0; loopn < numOfLasers; ++loopn) {
            // check the curves' parameter in difop
            checkbit =
                *(data + 50 + loopn * 15) ^ *(data + 50 + loopn * 15 + 1);
            for (int loopm = 1; loopm < 7; ++loopm) {
              checkbit = checkbit ^ (*(data + 50 + loopn * 15 + loopm * 2)) ^
                         (*(data + 50 + loopn * 15 + loopm * 2 + 1));
            }
            if (checkbit != *(data + 50 + loopn * 15 + 14)) {
              return;
            }
          }
          for (int loopn = 0; loopn < numOfLasers; ++loopn) {
            // calculate curves' parameters
            bit1 = static_cast<int>(*(data + 50 + loopn * 15));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 1));
            aIntensityCal[0][loopn] = (bit1 * 256 + bit2) * 0.001;
            bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 2));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 3));
            aIntensityCal[1][loopn] = (bit1 * 256 + bit2) * 0.001;
            bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 4));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 5));
            aIntensityCal[2][loopn] = (bit1 * 256 + bit2) * 0.001;
            bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 6));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 7));
            aIntensityCal[3][loopn] = (bit1 * 256 + bit2) * 0.001;
            bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 8));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 9));
            aIntensityCal[4][loopn] = (bit1 * 256 + bit2) * 0.00001;
            bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 10));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 11));
            aIntensityCal[5][loopn] = -(bit1 * 256 + bit2) * 0.0001;
            bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 12));
            bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 13));
            aIntensityCal[6][loopn] = (bit1 * 256 + bit2) * 0.001;
          }
          this->is_init_curve_ = true;
          std::cout << "this->is_init_curve_ = "
                    << "true!" << std::endl;
          Curvesis_new = true;
        }

        if ((data[290] != 0x00) && (data[290] != 0xff)) {
          intensityFactor = static_cast<int>(*(
              data + 290));  // intensity factor introduced since than 20181115
          // std::cout << intensityFactor << std::endl;
        }

        if ((data[291] == 0x00) || (data[291] == 0xff) || (data[291] == 0xa1)) {
          intensity_mode_ = 1;  // mode for the top firmware lower than
                                // T6R23V8(16) or T9R23V6(32) std::cout <<
                                // "intensity mode is 1" << std::endl;
        } else if (data[291] == 0xb1) {
          intensity_mode_ = 2;  // mode for the top firmware higher than
                                // T6R23V8(16) or T9R23V6(32) std::cout <<
                                // "intensity mode is 2" << std::endl;
        } else if (data[291] == 0xc1) {
          intensity_mode_ = 3;  // mode for the top firmware higher than T6R23V9
        }
      }
    }

    if (!this->is_init_angle_) {
      // check header
      if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 &&
          data[3] == 0x5a) {
        bool angle_flag = true;
        // check difop reigon has beed flashed the right data
        if ((data[1165] == 0x00 || data[1165] == 0xff) &&
            (data[1166] == 0x00 || data[1166] == 0xff) &&
            (data[1167] == 0x00 || data[1167] == 0xff) &&
            (data[1168] == 0x00 || data[1168] == 0xff)) {
          angle_flag = false;
        }
        // angle
        if (angle_flag) {
          // TODO check the HORI_ANGLE
          int bit1, bit2, bit3, symbolbit;
          for (int loopn = 0; loopn < numOfLasers; ++loopn) {
            if (loopn < 8 && numOfLasers == 16) {
              symbolbit = -1;
            } else {
              symbolbit = 1;
            }
            bit1 = static_cast<int>(*(data + 1165 + loopn * 3));
            bit2 = static_cast<int>(*(data + 1165 + loopn * 3 + 1));
            bit3 = static_cast<int>(*(data + 1165 + loopn * 3 + 2));
            VERT_ANGLE[loopn] =
                (bit1 * 256 * 256 + bit2 * 256 + bit3) * symbolbit * 0.01f;
            // std::cout << VERT_ANGLE[loopn] << std::endl;
            // TODO
            HORI_ANGLE[loopn] = 0;
          }
          this->is_init_angle_ = true;
          std::cout << "this->is_init_angle_ = "
                    << "true!" << std::endl;
        }
      }
    }
    // std::cout << "DIFOP data! +++++++++++++" << std::endl;
  }

  int correctAzimuth(float azimuth_f, int passageway) {
    int azimuth;
    if (azimuth_f > 0.0 && azimuth_f < 3000.0) {
      azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
    } else {
      azimuth_f = azimuth_f + HORI_ANGLE[passageway];
    }
    azimuth = (int)azimuth_f;
    azimuth %= 36000;

    return azimuth;
  }

  double getExactTime(int dsr, int firing) {
    return mRS16TimeBlock[firing][dsr];
  }

  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

  constexpr static const float ROTATION_RESOLUTION =
      0.01f; /**< degrees 旋转角分辨率*/
  static const uint16_t ROTATION_MAX_UNITS =
      36000; /**< hundredths of degrees */

  constexpr static const float DISTANCE_MAX = 200.0f;            /**< meters */
  constexpr static const float DISTANCE_MIN = 0.2f;              /**< meters */
  constexpr static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
  constexpr static const float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
  constexpr static const float DISTANCE_MAX_UNITS =
      (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;  //
  static const uint16_t LOWER_BANK = 0xddff;

  /** Special Defines for RS16 support **/
  static const int RS16_FIRINGS_PER_BLOCK = 2;
  static const int RS16_SCANS_PER_FIRING = 16;
  constexpr static const float RS16_BLOCK_TDURATION = 100.0f;  // [µs]
  constexpr static const float RS16_DSR_TOFFSET = 3.0f;        // [µs]
  constexpr static const float RS16_FIRING_TOFFSET = 50.0f;    // [µs]

  /** Special Defines for RS32 support **/
  static const int RS32_FIRINGS_PER_BLOCK = 1;
  static const int RS32_SCANS_PER_FIRING = 32;
  constexpr static const float RS32_BLOCK_TDURATION = 50.0f;  // [µs]
  constexpr static const float RS32_DSR_TOFFSET = 3.0f;       // [µs]
  constexpr static const float RS32_FIRING_TOFFSET = 50.0f;   // [µs]

  static const int TEMPERATURE_MIN = 31;
  static const int TEMPERATURE_RANGE = 40;

  /** \brief Raw rslidar data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  // block
  typedef struct raw_block {
    uint16_t header;  ///< UPPER_BANK or LOWER_BANK
    uint8_t rotation_1;
    uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get
                         /// 0-35999, divide by 100 to get degrees
    uint8_t data[BLOCK_DATA_SIZE];  // 96
  } raw_block_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union two_bytes {
    uint16_t uint;
    uint8_t bytes[2];
  };

  static const int PACKET_SIZE = 1248;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

  /** \brief Raw Rsldar packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  //    typedef struct raw_packet
  //    {
  //        //这儿是u8*42个的头
  //      uint8_t header[8];//
  //      uint8_t noneValue1[12];
  //      uint8_t year;//2000+year
  //      uint8_t month;
  //      uint8_t day;
  //      uint8_t hour;
  //      uint8_t minute;
  //      uint8_t second;
  //      uint16_t millisecond;
  //      uint16_t microsecond;
  //      uint8_t lidarModel;//  0x01 16  ，0x02 32
  //      uint8_t noneValue2[11];
  //      raw_block_t blocks[BLOCKS_PER_PACKET];//1200
  //      uint8_t status[6];// 6 byte
  //    } raw_packet_t;

  typedef struct raw_packet {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE];
  } raw_packet_t;

 protected:
  ModelType m_modelType;

  int numOfLasers;

  int64_t mRS16TimeBlock[2016][16];

  int tempPacketNum;

  float temper;

  int dis_resolution_mode;

  int VERT_ANGLE[32];

  float COS_VERT_ANGLE[32];

  float SIN_VERT_ANGLE[32];

  float HORI_ANGLE[32];

  float aIntensityCal[7][32];

  bool Curvesis_new = true;

  int g_ChannelNum[32][51];

  float CurvesRate[32];

  int intensity_mode_;

  int intensityFactor;

  bool is_init_curve_;

  bool is_init_angle_;

  bool is_init_top_fw_;

  /* cos/sin lookup table */
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;

  float Rx_;  // the optical center position in the lidar coordination in x
              // direction
  float Ry_;  // the optical center position in the lidar coordination in y
              // direction, for now not used
  float Rz_;  // the optical center position in the lidar coordination in z
              // direction

  int return_mode_;
};

}  // namespace liso

#endif
