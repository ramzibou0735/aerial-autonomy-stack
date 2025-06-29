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
- Run `nvidia-settings` and select "NVIDIA (Performance Mode)" under PRIME Profiles

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

# To avoid having to sudo the docker command:
# sudo groupadd docker
# sudo usermod -aG docker $USER
# newgrp docker # or logout/login
# docker run hello-world

# To add docker compose
# sudo apt-get update
# sudo apt-get install docker-compose-plugin
# docker compose version
```

## Use the Simulation Docker

...

## Use the Aircraft Docker

...

