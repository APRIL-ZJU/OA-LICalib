cmake_minimum_required(VERSION 2.8.3)
project(oa_licalib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

#SET(CXX_MARCH native)
#set(CMAKE_CXX_FLAGS "-ftemplate-backtrace-limit=0  -march=${CXX_MARCH} -O3 -DBASALT_USE_CHOLMOD -DBASALT_DISABLE_ASSERTS -DNDEBUG")
#set(CMAKE_CXX_FLAGS " -march=${CXX_MARCH} -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)
#add_definitions(-O3 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})
#set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen")
set(CERES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-ceres-slover")

#find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(STD_CXX_FS stdc++fs)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Found ceres headers in: ${CERES_INCLUDE_DIRS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    message_generation
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    tf
    eigen_conversions
    tf_conversions
    ndt_omp
    roslib
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
#include_directories(${YAML_CPP_INCLUDEDIR})
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

set(PANGOLIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-pangolin")
find_package(Pangolin REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  feature_cloud.msg
  imu_array.msg
  pose_array.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs sensor_msgs
#  LIBRARIES spline_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

include_directories(thirdparty/Sophus)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ceres
)

################ Library #######################
add_library(lib_calib 
  src/trajectory/se3_trajectory.cpp
  src/trajectory/trajectory_estimator.cpp
  src/trajectory/trajectory_viewer.cpp
  src/calib/lidar_localization.cpp
  src/calib/lidar_ndt_odometry.cpp
  src/calib/surfel_association.cpp
  src/calib/inertial_initializer.cpp
  src/calib/scan_undistortion.cpp
)
target_link_libraries(lib_calib ${thirdparty_libraries})
add_dependencies(lib_calib  ${PROJECT_NAME}_generate_messages_cpp)
################# Node ###################

add_executable(li_calib_node src/app/li_calib_node.cpp src/calib/calib_helper.cpp)
target_link_libraries(li_calib_node lib_calib ${thirdparty_libraries})
add_dependencies(li_calib_node  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(map_evaluation_tools src/app/map_evaluation_tools.cpp)
target_link_libraries(map_evaluation_tools ${thirdparty_libraries})

