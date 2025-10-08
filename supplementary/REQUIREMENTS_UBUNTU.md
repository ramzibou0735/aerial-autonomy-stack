# Pre-installation Steps for AAS on Ubuntu 22

## Install Ubuntu 22 with NVIDIA Driver

- Install the host OS from a startup disk based on `ubuntu-22.04.5-desktop-amd64.iso`
- Choose "Normal installation", "Download updates while installing Ubuntu", no "Install third-party software"
- Run "Software Updater", restart
- "Update All" in "Ubuntu Software" (including `killall snap-store && sudo snap refresh snap-store`)
- Update and restart for "Device Firmware" as necessary
- In "Software & Updates", select `nvidia-driver-580 (propietary, tested)`
- Running `nvidia-smi` will report Driver Version: 580.65.06, CUDA Version: 13.0

```sh
sudo apt update
sudo apt upgrade

# Select PRIME profile "NVIDIA (Performance Mode)" from CLI (or, if available, use `nvidia-settings` -> "PRIME Profiles" -> "NVIDIA (Performance Mode)")
sudo prime-select nvidia # Reboot and check in Ubuntu's "Settings" -> "About" -> "Graphics" is your NVIDIA card

sudo apt install mesa-utils
glxinfo | grep "OpenGL renderer" # Check the GPU is the OpenGL renderer
```

## Install Docker Engine and NVIDIA Container Toolkit

```sh
# Based on https://docs.docker.com/engine/install/ubuntu/ and https://docs.docker.com/engine/install/linux-postinstall/

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

# Install Docker Engine
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world # Test Docker is working
sudo docker version # Check version, 28.3.0 at the time of writing

# Remove the need to sudo the docker command
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker # Reboot

docker run hello-world # Test Docker is working without sudo
```

Log in to the NVIDIA Registry:

- Go to https://ngc.nvidia.com and login/create an account.
- Click on your account the top right, go to Setup -> Get API Key.
- Click "Generate API Key" -> "+ Generate Personal Key" for the "NCG Catalog" service, confirm, and copy the key.

```sh
docker login nvcr.io # To be able to reliably pull NVIDIA base images
Username: # type $oauthtoken
Password: # copy and paste the API key and press enter to pull base images from nvcr.io/
```

```sh
# Add NVIDIA Container Toolkit to use the GPU within containers
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

docker info | grep -i runtime # Check `nvidia` runtime is available

docker run --rm --gpus all nvcr.io/nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi # Test nvidia-smi works in a container with CUDA
```
