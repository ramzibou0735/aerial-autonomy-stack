# Rolling Notes

## Host Computer Setup

- Startup disk based on `ubuntu-22.04.5-desktop-amd64.iso`
- "Normal installation", "Download updates while installing Ubuntu", no "Install third-party software"
- Run "Software Updater", restart, Update All in "Ubuntu Software"
```sh
killall snap-store
sudo snap refresh snap-store
```
- Update and restart for "Device Firmware" as necessary
- In "Software & Updates", select `nvidia-driver-570 (propietary, tested)`
- `nvidia-smi` reports Driver Version: 570.133.07, CUDA Version: 12.8
- Run `nvidia-settings` and select "NVIDIA (Performance Mode)" under PRIME Profiles

```sh
sudo apt install mesa-utils # check GPU OpenGL renderer with $ glxinfo | grep "OpenGL renderer", also installed in the container, for gz sim GUI
```

## Host Computer Development Environment

```sh
sudo apt update
sudo apt upgrade
sudo apt install git

sudo dpkg -i code_1.101.2-1750797935_amd64.deb
chmod +x Anaconda3-2025.06-0-Linux-x86_64.sh 
./Anaconda3-2025.06-0-Linux-x86_64.sh
conda config --set auto_activate_base false

ssh-keygen 
cat ~/.ssh/id_rsa.pub 
mkdir ~/git
cd ~/git/
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git
```
- Docker Engine https://docs.docker.com/engine/install/ubuntu/, https://docs.docker.com/engine/install/linux-postinstall/ (skip Docker Compose for now)

```sh
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done # none should be there

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

```sh
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world
sudo docker version # 28.3.0 at the time of writing

# To avoid having to sudo the docker command: (skip for now)
# sudo groupadd docker
# sudo usermod -aG docker $USER
# newgrp docker # or logout/login
# docker run hello-world

# To add docker compose: (skip for now)
# sudo apt-get update
# sudo apt-get install docker-compose-plugin
# docker compose version
```

## Add NVIDIA Container Toolkit for GPU access within the container

```sh
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)  # e.g. ubuntu22.04
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Test with
```sh
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

## Docker Hygiene

```sh
sudo docker ps -a # list containers
sudo docker stop $(sudo docker ps -q) # stop all containers
sudo docker container prune # remove all containers
```

```sh
sudo docker images # list images
sudo docker rmi <image_name_or_id> # remove a specific image
sudo docker image prune # remove untagged images
```

## Base images

- Ubuntu images: https://hub.docker.com/_/ubuntu/tags
- NVIDIA L4T containers: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags
- NVIDIA Cuda containers: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags

```sh
FROM ubuntu:22.04 # For every computer
FROM nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04 # For host computers with NVIDIA GPU and installed 570 driver, run with --gpu all
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 # For NVIDIA Orin NX and JetPack 6
```

## Simulation Docker

```sh
sudo docker build -t simulation-image -f Dockerfile.simulation . # this takes about 15-20' from scratch
```

```sh
xhost +local:docker

sudo docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  --net=host \
  simulation-image

# It starts $ tmuxinator start -p /git/resources/simulation_tmuxinator.yml
# Move between windows with Ctrl + b, then n, p
# Move between panes with Ctrl + b, then arrows
# Detach with Ctrl + b, then press d
# Re-attach with $ tmux attach-session -t simulation_tmuxinator
# Or kill with $ tmux kill-session -t simulation_tmuxinator
# List sessions with $ tmux list-sessions

xhost -local:docker
```

`exit` or Ctrl+D will close the shell and stop the container if it was started interactively.
Ctrl + P  then  Ctrl + Q detaches you from the container and leaves it running in the background. Re-attach with `docker attach <container_name_or_id>`


## Aircraft Docker

```sh
sudo docker build -t aircraft-image -f Dockerfile.aircraft . # this takes about 15-20' from scratch
```

```sh
xhost +local:docker

sudo docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  --net=host \
  aircraft-image

# It starts $ tmuxinator start -p /git/resources/aircraft_tmuxinator.yml
# Move between windows with Ctrl + b, then n, p
# Move between panes with Ctrl + b, then arrows
# Detach with Ctrl + b, then press d
# Re-attach with $ tmux attach-session -t aircraft_tmuxinator
# Or kill with $ tmux kill-session -t aircraft_tmuxinator
# List sessions with $ tmux list-sessions

xhost -local:docker
```

`exit` or Ctrl+D will close the shell and stop the container if it was started interactively.
Ctrl + P  then  Ctrl + Q detaches you from the container and leaves it running in the background. Re-attach with `docker attach <container_name_or_id>`

## SITL Vehicles

### PX4

- https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html#x500-quadrotor-with-depth-camera-front-facing
- https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html#x500-quadrotor-with-2d-lidar 

### ArduPilot

- https://ardupilot.org/dev/docs/sitl-with-gazebo.html Iris quadcopter and a Zephyr delta-wing.
