# Pre-installation for WSL builds

> [!CAUTION]
> WSL support is work-in-progress
>
> AAS can be built and run in WSL with the following caveats:
> - In `simulation-image`, `gz sim`'s `ogre2` rendering is not GPU accelerated
> - In `aircraft-image`, `CUDAExecutionProvider` is not available
>
> This note will be removed once these issues are resolved

## Install WSL

The [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install) lets developers install a Linux distribution (e.g. Ubuntu, OpenSUSE, Arch Linux, etc.) and use Linux applications, utilities, and Bash command-line tools directly on Windows, unmodified, without the overhead of a traditional virtual machine or dualboot setup.

> [!NOTE]
> WSL2 is available on Windows 10 x64 (from version 1903/build 18362.1049) and Windows 11. 
> If `wsl --list` is not available, check the [manual installation steps](https://learn.microsoft.com/en-us/windows/wsl/install-manual).

- From PowerShell, check available Linux distributions `wsl --list --online`
- Install "Ubuntu-22.04" `wsl --install -d Ubuntu-22.04`
- Setup an account when prompted `Enter new UNIX username:` and `New password:`

```sh
# Install git
sudo apt update
sudo apt upgrade
sudo apt install git

# Install git-lfs (for the large files in simulation_resources/)
sudo apt install git-lfs
git lfs install

# Install xterm
sudo apt install xterm

# Install X11 apps and xserver
sudo apt install x11-apps
sudo apt install x11-xserver-utils
xclock # A new window with a clock should appear

# Create an ssh key (optional)
ssh-keygen 
cat ~/.ssh/id_rsa.pub
```

> [!IMPORTANT]
> When building and running large Docker images (e.g. the simulator and aircraft containers), WSL2 can easily consume available system resources. To prevent crashes, hangs, or 100% disk usage, configure WSL2’s resource limits using a `.wslconfig` file. 
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
> After editing `.wslconfig`, restart WSL for the new settings to take effect:
>
> ```sh
> exit
> wsl --shutdown 
> wsl ~
> free -h # To check available memory and swap
> ```
>
> For additional resources please see [advanced settings configuration](https://learn.microsoft.com/en-us/windows/wsl/wsl-config).

## NVIDIA Support in WSL

Download and install the NVIDIA recommended driver for you card using the [NVIDIA App](https://www.nvidia.com/en-us/software/nvidia-app/) 

>[!NOTE] 
>The latest NVIDIA Windows driver fully supports **WSL 2**, enabling existing CUDA applications compiled on Linux to run unmodified in WSL.
Once the Windows NVIDIA driver is installed, CUDA is available in WSL 2 via a stubbed `libcuda.so`. 
>
> **Do NOT install a separate NVIDIA GPU Linux driver inside WSL 2**,
> 
> The following instructions are tested on Driver Version: 571.59, CUDA Version 12.8

```sh
nvidia-smi # From WSL, check NVIDIA driver

sudo apt update
sudo apt install mesa-utils
glxinfo -B # You should see something like: `OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 4050 Laptop GPU)`
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

<!-- 

THIS SEEMS UNNECESSARY with WSLg 

## Install VcXsrv Windows X Server

WSL2 does not provide a full Linux desktop environment by default, so graphical applications (like Gazebo or other GUI tools) cannot run natively. To display GUI applications from WSL2 on Windows, we need an X server. VcXsrv is a lightweight and widely used X server for Windows.

- Download [VcXsrv:](https://sourceforge.net/projects/vcxsrv/) and download the installer.
- Run the installer with default settings.
    - Start “XLaunch” (comes with VcXsrv).
    - Choose Multiple windows.
    - Set Display number to 0.
    - Check Start no client.
    - Check Disable access control (or configure for security).
    - Finish to launch the X server.

From your WSL2 terminal, you should configure your `~/.bashrc` file to set up the display and X11 permissions automatically whenever you open a WSL terminal.

```sh
# WSL2 GUI setup
echo 'export DISPLAY=$(grep nameserver /etc/resolv.conf | awk "{print \$2}"):0' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_INDIRECT=0' >> ~/.bashrc
echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc
echo 'export __GLX_VENDOR_LIBRARY_NAME=nvidia' >> ~/.bashrc
echo 'if command -v xhost >/dev/null 2>&1; then xhost +local:; fi' >> ~/.bashrc

# Reload .bashrc
source ~/.bashrc
``` 

-->
