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
```
- ...
