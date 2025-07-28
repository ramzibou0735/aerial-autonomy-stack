#!/bin/bash

# Set up the aircraft
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
DRONE_ID="${DRONE_ID:-1}" # Id of aircraft (default = 1)
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false 

# TODO: turn off use_sim_time in aircraft.yml.erb

# Launch the aircraft container
docker run -d -t \
    # --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    # --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
    --env DRONE_ID=$DRONE_ID --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
    --privileged \
    --name aircraft-container \
    aircraft-image; \
exec bash"

echo "Now attach with: docker exec -it aircraft-container tmux attach"
