# Avionics

## Configure PX4's Network and DDS Client

> [!NOTE]
> Skip this step if you are using ArduPilot

- Access QGroundControl -> Analyze Tools -> MAVLink console
- Copy-and-paste the following commands (to assign an IP the PX4 autopilot (e.g., 10.10.1.33) and let the `uxrce_dds_client` connect to the NX (e.g., on IP 10.10.1.44) using namespace "Drone1")
- Re-start the autopilot 

```sh
mkdir /fs/microsd/etc
echo "uxrce_dds_client stop" > /fs/microsd/etc/extras.txt
echo "sleep 3" >> /fs/microsd/etc/extras.txt
echo "uxrce_dds_client start -p 8888 -h 10.10.1.44 -n Drone1" >> /fs/microsd/etc/extras.txt

echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo IPADDR=10.10.1.33 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg
echo ROUTER=10.10.1.254 >> /fs/microsd/net.cfg
echo DNS=10.10.1.254 >> /fs/microsd/net.cfg

# Check the content of the files
cat /fs/microsd/etc/extras.txt
cat /fs/microsd/net.cfg

netman update
```

Also read [PX4 documentation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/holybro_pixhawk_jetson_baseboard.md#ethernet-setup-using-netplan)

## Configure ArduPilot's MAVLink bridge

> [!NOTE]
> Skip this step if you are using PX4

MAVLink can be connected either over ethernet or using the Pixhawk 6X's TELEM2 serial port

- [Holybro documentation](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/mavlink-bridge)
- [ArduPilot documentation](https://ardupilot.org/copter/docs/common-serial-options.html)
- [PX4 documentation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/holybro_pixhawk_jetson_baseboard.md#mavlink-setup)

## Flash JetPack 6 to NVIDIA Orin

Holybro Jetson baseboard normally comes installed with JetPack 5, to upgrade, download NVIDIA SDK Manager on the Ubuntu 22 host computer from [this link](https://developer.nvidia.com/sdk-manager#installation_get_started)

```sh
cd ~/Downloads
sudo apt install ./sdkmanager_2.3.0-12617_amd64.deb 
sdkmanager # Log in with your https://developer.nvidia.com account 
```

- Put the Holybro Jetson baseboard in recovery mode with the dedicated switch
- Connect the USB-C port closes to the fan to the computer running `sdkmanager` and power on the board
- On Step 1, select "Jetson", "Host Machine Ubuntu 22 x86_64", "Target Hardware Jetson Orin NX" (automatically detected), "SDK Version JetPack 6.2.1"
- On Step 2, under "Target Components", select all "Jetson Linux" (uncheck all others)
- On the flash dialog after the download, choose "OEM Pre-config", username, password, and "Storage NVMe"
- Log in, finish the configuration, power-off, put the board out of recovery mode and power-on again

Also read [PX4 documentation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/holybro_pixhawk_jetson_baseboard.md#flashing-the-jetson-board)

> [!WARNING]
> At the time of writing, Snap is broken on JetPack 6, a fix is suggested [here](https://forums.developer.nvidia.com/t/chromium-other-browsers-not-working-after-flashing-or-updating-heres-why-and-quick-fix/338891)
> ```sh
> snap download snapd --revision=24724
> sudo snap ack snapd_24724.assert
> sudo snap install snapd_24724.snap
> sudo snap refresh --hold snapd
> 
> snap install firefox
> ```

## Install Docker Engine and NVIDIA Container Toolkit

```sh
# Install git
sudo apt update
sudo apt install git

# Install git-lfs (for the large files in simulation_resources/)
sudo apt install git-lfs
git lfs install

# Create an ssh key
ssh-keygen 
cat ~/.ssh/id_rsa.pub

# Login to the NVIDIA Registry
docker login nvcr.io # To be able to reliably pull NVIDIA base images
Username: # type $oauthtoken
Password: # copy and paste the API key and press enter to pull base images from nvcr.io/

# Add NVIDIA Container Toolkit for GPU use within containers
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
docker run --rm --runtime=nvidia nvidia/l4t-base:r36.2.0 nvidia-smi
```
