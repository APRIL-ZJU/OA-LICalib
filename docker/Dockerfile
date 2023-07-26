
FROM ros:melodic-ros-base
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update -y

# Install basic packages
RUN apt-get install -y build-essential
RUN apt-get install -y apt-utils
RUN apt-get install -y nautilus
RUN apt-get install -y nano
RUN apt-get install -y python-pip
RUN apt-get install -y curl
RUN apt-get install -y git
RUN apt-get install -y wget
RUN apt-get install -y gpg
RUN apt-get install -y ca-certificates
RUN apt-get install -y python-wstool

# Install external packages with apt
RUN apt-get install -y libgoogle-glog-dev
RUN apt-get install -y libglew-dev
RUN apt-get install -y libopencv-dev
RUN apt-get install -y python-wstool
RUN apt-get install -y libyaml-cpp-dev
RUN apt-get install -y libpcap-dev

# Install ROS packages
RUN apt-get update -y
RUN apt-get install -y ros-melodic-nav-msgs
RUN apt-get install -y ros-melodic-velodyne-msgs
RUN apt-get install -y ros-melodic-pcl-ros
RUN apt-get install -y ros-melodic-eigen-conversions
RUN apt-get install -y ros-melodic-tf-conversions
RUN apt-get install -y ros-melodic-angles
RUN apt-get install -y ros-melodic-diagnostic-updater
RUN apt-get install -y ros-melodic-cv-bridge

# Prepare OA-LICalib
RUN bash -c "source /opt/ros/melodic/setup.bash && mkdir -p ~/catkin_oa_calib/src && cd ~/catkin_oa_calib/src && catkin_init_workspace"

WORKDIR /root/catkin_oa_calib/src
RUN git clone https://github.com/APRIL-ZJU/OA-LICalib.git

RUN wstool init
RUN wstool merge OA-LICalib/depend_pack.rosinstall
RUN wstool update

WORKDIR /root/catkin_oa_calib/src/OA-LICalib
RUN git submodule sync --recursive
RUN git submodule update --init --recursive

# Install external packages from source
WORKDIR /root/catkin_oa_calib/src/OA-LICalib/thirdparty
RUN bash -c "cd ceres-solver && mkdir build && cd build && cmake .. && make -j12 && make install"
RUN bash -c "cd Pangolin && mkdir build && cd build && cmake .. && make -j12 && make install"
RUN bash -c "cd Sophus && mkdir build && cd build && cmake .. && make -j12 && make install"

RUN bash -c "echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc"

# Issue with flann and lz4
RUN mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
RUN mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak
RUN ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
RUN ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

WORKDIR /root/catkin_oa_calib
RUN bash -c "source /opt/ros/melodic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=\"ndt_omp\""
RUN bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

RUN bash -c "echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc"
RUN bash -c "echo 'alias sors=\"source devel/setup.bash\"' >> ~/.bashrc"