#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set up the aircraft
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
DRONE_ID="${DRONE_ID:-1}" # Id of aircraft (default = 1)
HEADLESS="${HEADLESS:-true}" # Options: true (default), false 
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false
SUBNET_PREFIX="${SUBNET_PREFIX:-42.42}" # Subnet prefix, e.g., 42.42 (default), 192.168, etc.

# Launch the aircraft container
docker run -it --rm \
  --runtime nvidia \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
  --env DRONE_TYPE=$DRONE_TYPE \
  --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env SIMULATED_TIME=true \
  --env SUBNET_PREFIX=$SUBNET_PREFIX \
  --env GST_DEBUG=3 \
  --net=host \
  --privileged \
  --name aircraft-container_$DRONE_ID \
  -v ~/tensorrt_cache/:/tensorrt_cache \
  aircraft-image
