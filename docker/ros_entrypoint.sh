#!/bin/bash
 
set -e
source "/opt/ros/melodic/setup.bash"

echo "================OA-LICalib Docker Env Ready================"

cd /root/catkin_ws

exec "$@"
