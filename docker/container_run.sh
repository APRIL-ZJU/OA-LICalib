#!/bin/bash
# Author: TaeYoung Kim (https://github.com/Taeyoung95)
# Edited: Jedidiah Alindogan (https://github.com/jedi-alindogan)

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR=$(dirname "$PWD")

# Move to the parent folder of the project directory
cd "$PROJECT_DIR"

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# Check if arguments are provided for the image name and tag
if [ "$#" -ne 2 ]; then
  echo "[Error] Usage: $0 <container_name> <image_name:tag>"
  exit 1
fi

# Assign the arguments to variables for clarity
CONTAINER_NAME="$1"
IMAGE_NAME="$2"

# Allow local docker container to connect to the X server on a linux machine
xhost +local:docker

# Launch the nvidia-docker container with the provided image name and tag
if [ "$(docker ps -aqf name=${CONTAINER_NAME})" ]; then
  echo "Starting the container ${CONTAINER_NAME} from ${IMAGE_NAME}..." 
  if [ ! "$(docker ps -qf name=${CONTAINER_NAME})" ]; then
    docker start "$CONTAINER_NAME"
  fi
  docker exec -it "$CONTAINER_NAME" /bin/bash

else
  echo "Starting a new container ${CONTAINER_NAME} from ${IMAGE_NAME}..."
  docker run --privileged -it \
           --gpus all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume="$PROJECT_DIR:/root/catkin_ws/src/OA-LICalib" \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=2gb \
           --name="$CONTAINER_NAME" \
           --env="DISPLAY=$DISPLAY" \
           "$IMAGE_NAME" /bin/bash
fi
