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
- ...
