#!/bin/bash

# Set up the simulation
DRONE_TYPE="quad" # Options: quad, vtol
AUTOPILOT="px4" # Options: px4, ardupilot
NUM_DRONES=2 # Number of aircraft to launch

# Grant access to the X server
xhost +local:docker

# Launch simulation container
gnome-terminal --geometry=120x36+10+10 -- bash -c "echo 'Launching Simulation Container...'; \
  docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
      --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
      --env NUM_DRONES=$NUM_DRONES \
    --privileged --net=host \
    --name simulation-container \
    simulation-image; \
  exec bash"

# Launch aircraft containers
for i in $(seq 1 $NUM_DRONES); do
  X_POS=$(( 1060 + (i) * 50 ))
  Y_POS=$(( 10 + (i-1) * 200 ))
  gnome-terminal --geometry=120x36+${X_POS}+${Y_POS} -- bash -c "echo 'Launching Aircraft Container $i...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
      --env ROS_DOMAIN_ID=$i --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
      --env DRONE_ID=$i \
      --privileged --net=host \
      --name aircraft-container_$i \
      aircraft-image; \
    exec bash"
done

# Launch htop in a new terminal
gnome-terminal --geometry=120x24+10+710 --title="System Monitor" -- htop

echo "Fly, my pretties, fly!"