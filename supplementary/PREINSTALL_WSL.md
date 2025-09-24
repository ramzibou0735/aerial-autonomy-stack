# Pre-installation for Windows 11

## Setup WSLg for Ubuntu 22

> [!NOTE]
> The [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install) lets developers install a Linux distribution (e.g. Ubuntu, OpenSUSE, Arch Linux, etc.) and use Linux applications, utilities, and Bash command-line tools directly on Windows, unmodified, without the overhead of a traditional virtual machine or dualboot setup
> 
> **The latest WSL2 and WSLg (WSL2 extension with GUI capabilities) are already included in Windows 11**

- From PowerShell, check available Linux distributions `wsl --list --online`
- Install "Ubuntu-22.04" `wsl --install -d Ubuntu-22.04`
- Setup an account when prompted `Enter new UNIX username:` and `New password:`

```sh
sudo apt update
sudo apt upgrade

# Install git
sudo apt install git

# Install git-lfs (for the large files in simulation_resources/)
sudo apt install git-lfs
git lfs install

# Install xterm
sudo apt install xterm
sudo apt install xfonts-base

# Install X11 apps and xserver
sudo apt install x11-apps
sudo apt install x11-xserver-utils
xclock # A new window with a clock should appear
```

> [!IMPORTANT]
> When building and running large Docker images (e.g. the simulator and aircraft containers), WSL can easily consume available system resources. To prevent crashes, hangs, or 100% disk usage, configure WSL’s resource limits using a `.wslconfig` file
> 
> - Create (or edit) the file `C:\Users\<YourWindowsUsername>\.wslconfig` (make sure it has no extension)
> - Add the following content to it (change `YourWindowsUsername`, increase the amount of resources, if available)
> 
> ```sh
> [wsl2]
> memory=12GB
> processors=8
> swap=16GB
> swapfile=C:\\Users\\<YourWindowsUsername>\\AppData\\Local\\Temp\\wsl-swap.vhdx
> localhostForwarding=true
> ```
>
> After editing `.wslconfig`, restart WSL from PowerShell for the new settings to take effect:
>
> ```sh
> exit
> wsl --shutdown 
> wsl ~
> free -h # To check available memory and swap
> ```

## Install the NVIDIA Driver in Windows 11

Download and install **NVIDIA driver 580** using the [NVIDIA App](https://www.nvidia.com/en-us/software/nvidia-app/) 

> [!NOTE] 
> The latest NVIDIA Windows drivers fully support **WSL2**, enabling existing CUDA applications compiled on Linux to run unmodified in WSL, once the Windows NVIDIA driver is installed, CUDA is available in WSL2 via a stubbed `libcuda.so`
>
> **Do NOT install a separate NVIDIA GPU Linux driver inside WSL2**

From PowerShell

```sh
wsl ~

nvidia-smi # From WSL, check NVIDIA driver
# These instructions are tested on Driver Version: 581.15, CUDA Version:13.0

sudo apt update
sudo apt install mesa-utils
glxinfo -B # You should see something like: `OpenGL renderer string: D3D12 (NVIDIA RTX A4500)`
```

## Install Docker Engine and NVIDIA Container Toolkit inside WSLg

From PowerShell

```sh
wsl ~

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

Log in to the NVIDIA Registry:

- Go to https://ngc.nvidia.com and login/create an account.
- Click on your account the top right, go to Setup -> Get API Key.
- Click "Generate API Key" -> "+ Generate Personal Key" for the "NCG Catalog" service, confirm, and copy the key.

```sh
docker login nvcr.io # To be able to reliably pull NVIDIA base images
Username: # type $oauthtoken
Password: # copy and paste (e.g. right-click once in PowerShell) the API key and press enter to pull base images from nvcr.io/
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
