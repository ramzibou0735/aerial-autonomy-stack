#!/bin/bash

# Set up the simulation
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
NUM_DRONES="${NUM_DRONES:-2}" # Number of aircraft (default = 2)
DEBUG="${DEBUG:-false}" # Whehther to launch the simulation or a simple shell (default = false)

# Grant access to the X server
xhost +local:docker

# Create network
docker network create --subnet=42.42.0.0/16 aas-network

ENTRYPOINT_FLAG=""
if [ "$DEBUG" = "true" ]; then
  ENTRYPOINT_FLAG="--entrypoint /bin/bash"
fi

# Launch the simulation container
gnome-terminal --geometry=120x36+10+10 -- bash -c "echo 'Launching Simulation Container...'; \
  docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
    --env NUM_DRONES=$NUM_DRONES \
    --net=aas-network --ip=42.42.1.99 \
    --privileged \
    --name simulation-container \
    ${ENTRYPOINT_FLAG} \
    simulation-image; \
  exec bash"

# Launch the aircraft containers
for i in $(seq 1 $NUM_DRONES); do
  X_POS=$(( 1060 + (i) * 50 ))
  Y_POS=$(( 10 + (i-1) * 200 ))
  gnome-terminal --geometry=120x36+${X_POS}+${Y_POS} -- bash -c "echo 'Launching Aircraft Container $i...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
      --env ROS_DOMAIN_ID=$i --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
      --env DRONE_ID=$i \
      --net=aas-network --ip=42.42.1.$i \
      --privileged \
      --name aircraft-container_$i \
      ${ENTRYPOINT_FLAG} \
      aircraft-image; \
    exec bash"
done

echo "Fly, my pretties, fly!"