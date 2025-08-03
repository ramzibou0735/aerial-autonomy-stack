# Pre-installation

## Install Ubuntu 22 with NVIDIA Driver

> [!NOTE]
> Skip this step if you already have an **Ubuntu 22 computer with NVIDIA Driver**, Git, **Git LFS** etc.

- Install the host OS from a startup disk based on `ubuntu-22.04.5-desktop-amd64.iso`
- Choose "Normal installation", "Download updates while installing Ubuntu", no "Install third-party software"
- Run "Software Updater", restart
- "Update All" in "Ubuntu Software" (including `$ killall snap-store && sudo snap refresh snap-store`)
- Update and restart for "Device Firmware" as necessary
- In "Software & Updates", select `nvidia-driver-575 (propietary, tested)`
- Running `$ nvidia-smi` will report Driver Version: 575.64.03, CUDA Version: 12.9
- Run `$ nvidia-settings` and select "NVIDIA (Performance Mode)" under "PRIME Profiles"

```sh
sudo apt install mesa-utils # Also installed in the simulation container, for gz sim rendering
# Check GPU OpenGL renderer
glxinfo | grep "OpenGL renderer"

# Install git
sudo apt update
sudo apt upgrade
sudo apt install git

# Install git-lfs (for the large files in simulation_resources/)
sudo apt install git-lfs
git lfs install

# Create an ssh key
ssh-keygen 
cat ~/.ssh/id_rsa.pub
```

## Install Docker Engine and NVIDIA Container Toolkit

> [!NOTE]
> Skip this step if you already installed **Docker Engine and NVIDIA Container Toolkit**

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
```

```sh
# Install and test Docker Engine
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world
sudo docker version # 28.3.0 at the time of writing

# Remove the need to sudo the docker command
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker # Reboot
docker run hello-world
```

Log In to the NVIDIA Registry:

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

# Check `nvidia` runtime is available
docker info | grep -i runtime

# Test with
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```
