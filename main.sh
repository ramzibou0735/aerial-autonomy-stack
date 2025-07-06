#!/bin/bash

# Set the number of aircraft containers to launch
AIRCRAFT_COUNT=1

# Grant access to the X server
xhost +local:docker

# Launch simulation container
gnome-terminal -- bash -c "echo 'Launching Simulation Container...'; \
  docker run -it --rm \
    --name sim_container \
    --net=host \
    --privileged \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri \
    --gpus all \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    simulation-image; \
  exec bash"

# Launch aircraft containers
for i in $(seq 1 $AIRCRAFT_COUNT); do
  gnome-terminal -- bash -c "echo 'Launching Aircraft Container $i...'; \
    docker run -it --rm \
      --name aircraft_container_$i \
      --net=host \
      --privileged \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
      --device /dev/dri \
      --gpus all \
      --env DISPLAY=$DISPLAY \
      --env QT_X11_NO_MITSHM=1 \
      aircraft-image; \
    exec bash"
done

# Launch htop in a new terminal
gnome-terminal --title="System Monitor" -- htop

echo "Launched all terminals."