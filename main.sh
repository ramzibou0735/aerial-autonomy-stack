#!/bin/bash

# Set up the simulation
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
NUM_DRONES="${NUM_DRONES:-2}" # Number of aircraft (default = 2)
WORLD="${WORLD:-impalpable_greyness}" # Options: impalpable_greyness (default), apple_orchard, shibuya_crossing, swiss_town
MODE="${MODE:-}" # Options: empty (default), debug, ...

# Initialize an empty variable for the flags
MODE_OPTS=""
case "$MODE" in
  debug)
    MODE_OPTS="--entrypoint /bin/bash"
    ;;
  example_mode)
    MODE_OPTS="--example-flag"
    ;;
  *)
    MODE_OPTS=""
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
    --env NUM_DRONES=$NUM_DRONES --env WORLD=$WORLD \
    --net=aas-network --ip=42.42.1.99 \
    --privileged \
    --name simulation-container \
    ${MODE_OPTS} \
    simulation-image; \
  exec bash"

# Launch the aircraft containers
for i in $(seq 1 $NUM_DRONES); do
  gnome-terminal --geometry=$(get_quadrant_geometry $(( i % 4 ))) -- bash -c "echo 'Launching Aircraft Container $i...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
      --env ROS_DOMAIN_ID=$i --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
      --env DRONE_ID=$i \
      --net=aas-network --ip=42.42.1.$i \
      --privileged \
      --name aircraft-container_$i \
      ${MODE_OPTS} \
      aircraft-image; \
    exec bash"
done

echo "Fly, my pretties, fly!"