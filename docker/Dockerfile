# Author: Jedidiah Alindogan (https://github.com/jedi-alindogan)

FROM ros:melodic

# Install OA-LiCALIB dependencies
RUN apt-get update && apt-get -y install \
    cmake ccache libeigen3-dev libpcl-dev \
    libcap-dev libyaml-cpp-dev libeigen3-dev \
    python2.7 python-wstool \
    wget

# Install Ceres-Solver dependencies
RUN apt-get -y install \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libmetis-dev

# Install ROS-related dependencies 
RUN apt-get -y install \  
    ros-melodic-angles ros-melodic-cv-bridge ros-melodic-diagnostic-updater \
    ros-melodic-eigen-conversions ros-melodic-pcl-conversions \
    ros-melodic-pcl-ros ros-melodic-sophus \
    ros-melodic-tf ros-melodic-tf-conversions \
    ros-melodic-velodyne ros-melodic-velodyne-msgs \
    ros-melodic-rslidar \
    ros-melodic-rslidar-driver ros-melodic-rslidar-msgs \ 
    ros-melodic-rslidar-pointcloud \
    ros-melodic-rviz

# Download Ceres-Solver, Sophus, and Pangolin with dependencies
WORKDIR /opt/local/lib/
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz \
    && tar zxf 2.1.0.tar.gz \
    && rm -rf 2.1.0.tar.gz \
    && mv ceres-solver* ceres-solver \
    && wget https://github.com/stevenlovegrove/Pangolin/archive/3f4a8b8.tar.gz \
    && tar zxf 3f4a8b8.tar.gz \
    && rm -rf 3f4a8b8.tar.gz \
    && mv Pangolin* Pangolin\
    && chmod 755 Pangolin/scripts/install_prerequisites.sh \
    && yes | ./Pangolin/scripts/install_prerequisites.sh -v recommended \ 
    && wget https://github.com/strasdat/Sophus/archive/00f3fd9.tar.gz \
    && tar zxf 00f3fd9.tar.gz \
    && rm -rf 00f3fd9.tar.gz \
    && mv Sophus* Sophus 
    

# Fix PCL and ROS lz4 name collision
RUN mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak && \
    mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4hc.h.bak && \
    ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h && \
    ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

# Create a catkin workspace
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'

WORKDIR /root/catkin_ws
COPY ./setup.sh .
RUN chmod 755 ./setup.sh .

# Load the ROS environment
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]



