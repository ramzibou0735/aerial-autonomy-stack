# Pre-installation for WSL builds

## Install WSL

The [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install) lets developers install a Linux distribution (such as Ubuntu, OpenSUSE, Kali, Debian, Arch Linux, etc) and use Linux applications, utilities, and Bash command-line tools directly on Windows, unmodified, without the overhead of a traditional virtual machine or dualboot setup.

> [!NOTE]
> These instructions are for Windows 10 version 2004 and higher (Build 19041 and higher) or Windows 11. For earlier versions, please see the manual [install page](https://learn.microsoft.com/en-us/windows/wsl/install-manual).

- From PowerShell, check available Linux distributions `wsl --list --online`
- Install "Ubuntu-22.04" `wsl --install -d Ubuntu-22.04`
- Enter new `UNIX username` and `Password`

```sh
# Install git
sudo apt update
sudo apt upgrade
sudo apt install git

# Install git-lfs (for the large files in simulation_resources/)
sudo apt install git-lfs
git lfs install

# Install xterm
sudo apt update
sudo apt install xterm -y

# Create an ssh key (optional)
ssh-keygen 
cat ~/.ssh/id_rsa.pub
```

> [!IMPORTANT]
> When building and running large Docker images (e.g. the simulator and aircraft containers), WSL2 can easily consume available system resources. To prevent crashes, hangs, or 100% disk usage, configure WSL2’s resource limits using a `.wslconfig` file. 
> 
> ```sh
> # From **Windows user home directory**: 
> # Create (or edit) the file `C:\Users\<YourWindowsUsername>\.wslconfig` to add:
>
> [wsl2]
> memory=10GB
> processors=8
> swap=16GB
> swapfile=C:\\Users\\<YourWindowsUsername>\\AppData\\Local\\Temp\\wsl-swap.vhdx
> localhostForwarding=true
>
> # After editing .wslconfig, restart WSL for the new settings to take effect:
> wsl --shutdown 
>
> # To check available memory and swap, open your WSL2 distro and run:
> free -h
> ```
> For additinal resources please see [advanced settings configuration](https://learn.microsoft.com/en-us/windows/wsl/wsl-config).

## Enable NVIDIA CUDA on WSL

Download and install the [NVIDIA CUDA enabled driver for WSL](https://www.nvidia.com/download/index.aspx) 

>[!NOTE] 
>The latest NVIDIA Windows GPU driver fully supports **WSL 2**, enabling existing CUDA applications compiled on Linux to run unmodified in WSL.
Once the Windows NVIDIA GPU driver is installed, CUDA is available in WSL 2 via a stubbed `libcuda.so`. 
>
> **Do NOT install a separate NVIDIA GPU Linux driver inside WSL 2**,

On **WSL 2 with NVIDIA GPU driver installed**, OpenGL is accelerated through **DirectX + GPU driver interop** (via WSLg).

```sh
# From WSL, check NVIDIA GPU
nvidia-smi

# Install Mesa utilities, for gz sim rendering 
sudo apt update
sudo apt install -y mesa-utils
glxinfo | grep "OpenGL renderer"

# If GPU acceleration is working, you should see something like: `OpenGL renderer string: D3D12 (Intel(R) UHD Graphics)`

# For CUDA workloads, WSL doesn’t rely on `glxinfo`. Instead it uses the NVIDIA Windows driver stubbed inside WSL (`libcuda.so`). So you can still have full CUDA even though OpenGL shows Intel. This only shows the renderer used for graphics (OpenGL), **not CUDA compute**.
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
echo 'export LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc
echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc
echo 'if command -v xhost >/dev/null 2>&1; then xhost +local:; fi' >> ~/.bashrc

# Reload .bashrc
source ~/.bashrc
```
