#!/bin/bash
set -e

# Set up the simulation
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
NUM_DRONES="${NUM_DRONES:-1}" # Number of aircraft (default = 1)
WORLD="${WORLD:-impalpable_greyness}" # Options: impalpable_greyness (default), apple_orchard, shibuya_crossing, swiss_town
HEADLESS="${HEADLESS:-false}" # Options: true, false (default)
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false 
MODE="${MODE:-}" # Options: empty (default), dev, ...

# Initialize an empty variable for the flags
MODE_SIM_OPTS=""
MODE_AIR_OPTS=""
if [[ "$MODE" == "dev" ]]; then
    SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    MODE_SIM_OPTS="--entrypoint /bin/bash"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_resources/:/simulation_resources:cached"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src:/ros2_ws/src:cached"
    MODE_AIR_OPTS="--entrypoint /bin/bash"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_resources/:/aircraft_resources:cached"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_ws/src:/ros2_ws/src:cached"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src/ground_system_msgs:/ros2_ws/src/ground_system_msgs:cached"
fi

# Grant access to the X server
export DISPLAY=${DISPLAY:-$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0}
export LIBGL_ALWAYS_INDIRECT=1
export QT_X11_NO_MITSHM=1
if command -v xhost >/dev/null 2>&1; then xhost +local:; fi

# Create network
NETWORK_NAME="aas-network"
docker network inspect "$NETWORK_NAME" >/dev/null 2>&1 || docker network create "$NETWORK_NAME"

# Launch the simulation container
xterm -T "Simulation" -e "docker run -it --rm \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
  --env NUM_DRONES=$NUM_DRONES --env WORLD=$WORLD --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env SIMULATED_TIME=true \
  --net $NETWORK_NAME --ip 42.42.1.99 --privileged --name simulation-container \
  $MODE_SIM_OPTS simulation-image; bash"

# Launch the aircraft containers
for i in $(seq 1 $NUM_DRONES); do
    sleep 1.5
    xterm -T "Aircraft $i" -e "docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
      --env ROS_DOMAIN_ID=$i --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
      --env DRONE_ID=$i --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
      --env SIMULATED_TIME=true \
      --net $NETWORK_NAME --ip 42.42.1.$i --privileged --name aircraft-container_$i \
      $MODE_AIR_OPTS aircraft-image; bash"
done

echo "Fly, my pretties, fly!"
