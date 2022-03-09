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

#include <ros/ros.h>
#include <utils/map_evaluation_utils.h>

using namespace std;

int main( int argc, char** argv ) {
  ros::init(argc, argv, "map_evaluation_tools");
  ros::NodeHandle nh("~");
  std::string map_path;
  int step_size;
  double radius;
  bool punish_solitar_points;
  int min_neighbors;

  nh.param<std::string>("map_path", map_path, " ");
  nh.param<int>("step_size", step_size, 1);
  nh.param<double>("radius", radius, 0.3);
  nh.param<int>("min_neighbors", min_neighbors, 15);
  nh.param<bool>("punish_solitar_points", punish_solitar_points, false);

  MapEvaluationTool met(map_path, step_size, radius, min_neighbors, punish_solitar_points);
  met.Process();

  return 0;
}
