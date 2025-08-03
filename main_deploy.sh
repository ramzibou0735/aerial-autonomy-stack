#!/bin/bash

# Set up the aircraft
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
DRONE_ID="${DRONE_ID:-1}" # Id of aircraft (default = 1)
HEADLESS="${HEADLESS:-true}" # Options: true (default), false 
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false
MODE="${MODE:-}" # Options: empty (default), dev, ...

# Initialize an empty variable for the flags
MODE_OPTS=""
case "$MODE" in
  dev)
    MODE_OPTS="--entrypoint /bin/bash"
    ;;
  *)
    MODE_OPTS=""
    ;;
esac

if [ "$HEADLESS" = "false" ]; then
    # Grant access to the X server
    xhost +local:docker # Remove this when building TensorRT cache for the first time
fi

# Launch the aircraft container in detached mode
docker run -d -t \
    --runtime nvidia \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
    --env DRONE_ID=$DRONE_ID --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
    --env SIMULATED_TIME=false --env HEADLESS=$HEADLESS\
    --net=host \
    --privileged \
    --name aircraft-container \
    -v ~/tensorrt_cache/:/tensorrt_cache \
    ${MODE_OPTS} \
    aircraft-image

echo "Now attach with: docker exec -it aircraft-container tmux attach"
echo "If MODE=dev, attach with: docker exec -it aircraft-container bash"

# docker stop $(docker ps -q) # Stop all containers
# docker container prune # Remove stopped containers
