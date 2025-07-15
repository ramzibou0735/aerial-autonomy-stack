# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

- **Develop** drone autonomy with ROS2
- **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
- **Deploy** in real drones with NVIDIA Orin/JetPack

> [!NOTE]
> For the motivation behind AAS (and how it compares to similar projects), read [`RATIONALE.md`](/docs/RATIONALE.md)

## Feature Highlights

- Support for **multiple quadrotor and VTOL** aircraft based on **PX4 or ArduPilot**
- Support for **ROS2** with ROS2-based autopilot interfaces (*via* XRCE-DDS and MAVSDK)
- Support for **YOLOv8** and ONNX CPU, CUDA (on desktop/amd64), and TensorRT (on Orin/arm64) Runtime
- Photogrammetry worlds for **software-in-the-loop simulation**
- Dockerized simulation based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)

- Support for [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (Fast LiDAR-Inertial Odometry)
- Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode in CTBR (`VehicleRatesSetpoint`) for agile, GNSS-denied flight
- Lightweight inter-drone communication for real-world deployment 

> [!NOTE]
> AAS leverages the following frameworks:
>
> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.15*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVSDK](https://github.com/mavlink/mavsdk/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 6/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 6/2025)

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [`nvidia-driver-570`](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 6/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060 computers
> 
> **To setup (i) Ubuntu 22, (ii) NVIDIA driver, (iii) Docker Engine, (iv) NVIDIA Container Toolkit, and (v) NVIDIA NGC API Key read [`PREINSTALL.md`](/docs/PREINSTALL.md)**

```sh
# Clone this repo
mkdir -p ~/git
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git ~/git/aerial-autonomy-stack
cd ~/git/aerial-autonomy-stack
```

### Option 1:  Build the Docker Images

> [!WARNING]
> Building from scratch requires a stable internet connection, `Ctrl + c` and restart if needed 

```sh
docker build -t simulation-image -f docker/Dockerfile.simulation . # The first build takes ~15' and creates an 18GB image (6GB for ros-humble-desktop, 9GB for PX4 and ArduPilot SITL)

docker build -t aircraft-image -f docker/Dockerfile.aircraft . # Having built Dockerfile.simulation, the first build takes ~15' and creates a 16GB image (6GB for ros-humble-desktop, 7GB for YOLOv8, ONNX)
```

These are development-friendly images with lots of tools and artifacts, trim if needed

### Option 2: Pull the Pre-built Docker Images

```sh
# TODO add .github workflow to build and push the images
docker pull jacopopan/simulation-image:latest # TODO
docker pull jacopopan/aircraft-image:latest # TODO
```

---

## Part 2: Simulation with AAS

```sh
cd ~/git/aerial-autonomy-stack/
chmod +x ./main.sh
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=2 ./main.sh
```

> [!TIP]
> Tmux shortcuts:
> - Move between windows with `Ctrl + b`, then `n`, `p`
> - Move between panes with `Ctrl + b`, then `arrow keys`
> - Enter copy mode to scroll back with `Ctrl + [`, then `arrow keys`, exit with `q`
> - Detach with `Ctrl + b`, then press `d`
> - Re-attach with `$ tmux attach-session -t simulation_tmuxinator`
> - Or kill with `$ tmux kill-session -t simulation_tmuxinator`
> - List sessions with `$ tmux list-sessions`
> - Kill all with `$ tmux kill-server`
> 
> Docker hygiene:
```sh
docker ps -a # List containers
docker stop $(docker ps -q) # Stop all containers
docker container prune # Remove all stopped containers

docker images # List images
docker image prune # Remove untagged images
docker rmi <image_name_or_id> # Remove a specific image
```

---

## Part 3: Development with AAS

```sh
cd ~/git/aerial-autonomy-stack/
chmod +x ./main.sh
MODE=debug ./main.sh
```

---

## Part 4: Deployment of AAS

TBD

---
> You've done a man's job, sir. I guess you're through, huh?




















## TODOs

### Geospatial and Photogrammetry Resources

- https://support.pix4d.com/hc/en-us/articles/360000235126#OPF2
- https://visicomdata.com/samples
- https://github.com/domlysz/BlenderGIS
- https://cesium.com/platform/cesiumjs/ 
- https://app.gazebosim.org/fuel/worlds
- https://aszabo.substack.com/p/zero-to-hero-creating-gazebo-worlds?utm_campaign=post&utm_medium=web
- https://github.com/AndrejOrsula/space_robotics_gz_envs

- https://github.com/softwareunderground/awesome-open-geoscience?tab=readme-ov-file
- https://github.com/sacridini/Awesome-Geospatial
- https://github.com/awesome-photogrammetry/awesome-photogrammetry?tab=readme-ov-file

### Resources

- PX$ SITL architecture: https://docs.px4.io/main/en/simulation/#sitl-simulation-environment
- PX4 XRCE-DDS architecture: https://docs.px4.io/main/en/middleware/uxrce_dds.html#architecture

- ArduPilot SITL architecture: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture
- ArduPilot UARTs: https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html
- ArduPilot SITL models: https://github.com/ArduPilot/SITL_Models

```sh
find . -path ./.git -prune -o -type f -not -name "*.dae" -not -name "*.png" -not -name "*.stl" | xargs wc -l
find . -path ./.git -prune -o -type f -not -name "*.dae" -not -name "*.png" -not -name "*.stl" -not -name "*.sdf" | xargs wc -l
```
