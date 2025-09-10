#!/bin/bash

# Set up the simulation
NUM_QUADS="${NUM_QUADS:-1}" # Number of quadcopters (default = 1)
NUM_VTOLS="${NUM_VTOLS:-0}" # Number of VTOLs (default = 0)
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
WORLD="${WORLD:-impalpable_greyness}" # Options: impalpable_greyness (default), apple_orchard, shibuya_crossing, swiss_town
HEADLESS="${HEADLESS:-false}" # Options: true, false (default)
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false 
MODE="${MODE:-}" # Options: empty (default), dev, ...

# Initialize an empty variable for the flags
MODE_SIM_OPTS=""
MODE_AIR_OPTS=""
case "$MODE" in
  dev)
    # In dev mode, resources and workspaces are mounted from the host
    SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    MODE_SIM_OPTS="--entrypoint /bin/bash"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_resources/:/simulation_resources"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src:/ros2_ws/src"
    MODE_AIR_OPTS="--entrypoint /bin/bash"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_resources/:/aircraft_resources"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_ws/src:/ros2_ws/src"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src/ground_system_msgs:/ros2_ws/src/ground_system_msgs"
    ;;
  *)
    MODE_SIM_OPTS=""
    MODE_AIR_OPTS=""
    ;;
esac

# Grant access to the X server
xhost +local:docker

# Create network
docker network create --subnet=42.42.0.0/16 aas-network

# Helper to place the terminals on-screen
CHAR_WIDTH=20 # Adjust to your terminal size
CHAR_HEIGHT=40 # Adjust to your terminal size
SCREEN_GEOMETRY=$(xdpyinfo | grep dimensions | sed -r 's/^[^0-9]*([0-9]+x[0-9]+).*$/\1/')
SCREEN_WIDTH=$(echo $SCREEN_GEOMETRY | cut -d'x' -f1)
SCREEN_HEIGHT=$(echo $SCREEN_GEOMETRY | cut -d'x' -f2)
get_quadrant_geometry() {
    local index=$1
    local width_chars=$(( (SCREEN_WIDTH / 2) / CHAR_WIDTH ))
    local height_chars=$(( (SCREEN_HEIGHT / 2) / CHAR_HEIGHT ))
    local x_pos=$(( (index == 1 || index == 2) * (SCREEN_WIDTH / 2) ))
    local y_pos=$(( (index == 2 || index == 3) * (SCREEN_HEIGHT / 2) ))
    echo "${width_chars}x${height_chars}+${x_pos}+${y_pos}"
}

# Launch the simulation container
gnome-terminal --geometry=$(get_quadrant_geometry 0) -- bash -c "echo 'Launching Simulation Container...'; \
  docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
    --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS \
    --env WORLD=$WORLD --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
    --env SIMULATED_TIME=true \
    --net=aas-network --ip=42.42.1.99 \
    --privileged \
    --name simulation-container \
    ${MODE_SIM_OPTS} \
    simulation-image; \
  exec bash"

# Initialize a counter for the drone IDs
DRONE_ID=0

# Launch the quad containers
for i in $(seq 1 $NUM_QUADS); do
  DRONE_ID=$((DRONE_ID + 1))
  sleep 1.5 # Limit resource usage
  gnome-terminal --geometry=$(get_quadrant_geometry $(( DRONE_ID % 4 ))) -- bash -c "echo 'Launching Quadcopter Container $DRONE_ID...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
      --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
      --env DRONE_TYPE=quad \
      --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
      --env SIMULATED_TIME=true \
      --net=aas-network --ip=42.42.1.$DRONE_ID \
      --privileged \
      --name aircraft-container_$DRONE_ID \
      ${MODE_AIR_OPTS} \
      aircraft-image; \
    exec bash"
done

# Launch the vtol containers
for i in $(seq 1 $NUM_VTOLS); do
  DRONE_ID=$((DRONE_ID + 1))
  sleep 1.5 # Limit resource usage
  gnome-terminal --geometry=$(get_quadrant_geometry $(( DRONE_ID % 4 ))) -- bash -c "echo 'Launching VTOL Container $DRONE_ID...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
      --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
      --env DRONE_TYPE=vtol \
      --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
      --env SIMULATED_TIME=true \
      --net=aas-network --ip=42.42.1.$DRONE_ID \
      --privileged \
      --name aircraft-container_$DRONE_ID \
      ${MODE_AIR_OPTS} \
      aircraft-image; \
    exec bash"
done

echo "Fly, my pretties, fly!"
