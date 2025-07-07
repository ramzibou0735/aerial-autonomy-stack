#!/bin/bash

# Set up the simulation
DRONE_TYPE="quad" # Options: quad, vtol
AP="ardupilot" # Options: ardupilot, px4
NUM_DRONES=3 # Number of aircraft to launch

# Grant access to the X server
xhost +local:docker

# Launch simulation container
gnome-terminal -- bash -c "echo 'Launching Simulation Container...'; \
  docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
    --privileged --net=host \
    --name simulation-container \
    simulation-image; \
  exec bash"

# Launch aircraft containers
for i in $(seq 1 $NUM_DRONES); do
  gnome-terminal -- bash -c "echo 'Launching Aircraft Container $i...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
      --privileged --net=host \
      --name aircraft-container_$i \
      aircraft-image; \
    exec bash"
done

# Launch htop in a new terminal
gnome-terminal --title="System Monitor" -- htop

echo "Fly, my pretties, fly!"