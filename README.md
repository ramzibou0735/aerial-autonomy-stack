# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

- **Develop** drone autonomy with ROS2
- **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
- **Deploy** in real drones with NVIDIA Orin/JetPack

## Feature Highlights

- Support for **multiple quadrotors and VTOLs** based on **PX4 or ArduPilot**
- **ROS2**-based autopilot interfaces (*via* XRCE-DDS and MAVSDK)
- Support for **YOLOv8** and ONNX GPU Runtimes
- Support for **LiDAR-Inertial Odometry** ([TBD](https://github.com/))
- **Dockerized simulation and deployment** based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags), [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)

<details>
<summary><b>Additional Features:</b> <i>(expand)</i></summary>

- 3D worlds for [PX4](https://docs.px4.io/main/en/simulation/#sitl-simulation-environment)/[ArduPilot](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture) **software-in-the-loop (SITL) simulation**
- Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode in CTBR (`VehicleRatesSetpoint`) for agile, GNSS-denied flight 
- Steppable simulation interface for reinforcement learning 
- Lightweight inter-drone communication for real-world deployment 

</details>

> [!NOTE]
> <details>
> <summary>AAS leverages the following frameworks: <i>(expand)</i></summary>
> 
> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.15*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVSDK](https://github.com/mavlink/mavsdk/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 6/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 6/2025)
> </details>
> 
> For the motivation behind AAS and how it compares to similar projects, read [`RATIONALE.md`](/docs/RATIONALE.md)

<!-- [![Teaser](docs/assets/video.jpg)](https://www.youtube.com/watch?v=VIDEO_ID) -->

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [`nvidia-driver-575`](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 7/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060 computers
> 
> **To setup (i) Ubuntu 22, Git LFS, (ii) NVIDIA driver, (iii) Docker Engine, (iv) NVIDIA Container Toolkit, and (v) NVIDIA NGC API Key, read [`PREINSTALL.md`](/docs/PREINSTALL.md)**

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
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=2 WORLD=swiss_town HEADLESS=false ./main.sh # Read main.sh for more options
```

> Once "Ready to Fly", one can takeoff and control from QGroundControl's ["Fly View"](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html). E.g., for PX4 VTOL, takeoff -> change altitude -> transition to FW, then take manual control; for ArduPilot VTOL, change mode to FBW A -> arm -> throttle all the way up -> then change mode to Loiter.

![Worlds](docs/assets/worlds.jpg)

Available `WORLD`s:
- `apple_orchard`, a GIS world created using [BlenderGIS](https://github.com/domlysz/BlenderGIS)
- `impalpable_greyness`, (default) an empty world with simple shapes
- `shibuya_crossing`, a 3D world adapted from [cgtrader](https://www.cgtrader.com/)
- `swiss_town`, a photogrammetry world courtesy of [Pix4D / pix4d.com](https://support.pix4d.com/hc/en-us/articles/360000235126)

> [!TIP]
> <details>
> <summary>Tmux and Docker Shortcuts <i>(expand)</i></summary>
> 
> - Move between Tmux windows with `Ctrl + b`, then `n`, `p`
> - Move between Tmux panes with `Ctrl + b`, then `arrow keys`
> - Enter copy mode to scroll back with `Ctrl + [`, then `arrow keys`, exit with `q`
> - Detach Tmux with `Ctrl + b`, then press `d`
> ```sh
> tmux list-sessions # List all sessions
> tmux attach-session -t [session_name] # Reattach a session
> tmux kill-session -t [session_name] # Kill a session
> tmux kill-server # Kill all sessions
> ```
> Docker hygiene:
> ```sh
> docker ps -a # List containers
> docker stop $(docker ps -q) # Stop all containers
> docker container prune # Remove all stopped containers
> 
> docker images # List images
> docker image prune # Remove untagged images
> docker rmi <image_name_or_id> # Remove a specific image
> ```
> 
> </details>

### Steppable Simulation

TBD

```sh
gz service -s /world/impalpable_greyness/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'multi_step: 250, pause: true'
```

---

## Part 3: Development with AAS

TBD: mount, edit, build, push back to git

```sh
cd ~/git/aerial-autonomy-stack/
chmod +x ./main.sh
MODE=debug ./main.sh
```

### Run an Example

```sh
# on vehicle 1
./skibidi.sh
```

---

## Part 4: Deployment of AAS

TBD

> [!IMPORTANT]
> **To setup PX4 parameters and DDS client, read [`PX4_SETUP.md`](/docs/PX4_SETUP.md)**

```sh
# run on vehicle with
docker run -d -t --name 
docker exec -it <container_name_or_id> tmux attach
```

---
> You've done a man's job, sir. I guess you're through, huh?

<!-- 



## TODOs

### LIO

- https://github.com/PRBonn/kiss-icp

- https://github.com/hku-mars/FAST_LIO
- https://github.com/Ericsii/FAST_LIO_ROS2
- https://github.com/Taeyoung96/FAST_LIO_ROS2

### Model Resources

- ArduPilot SITL models: https://github.com/ArduPilot/SITL_Models

### Dependencies Issues

- In yolo.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumbent OpenCV)
- Gazebo Sim versioning is a disaster, but consider Jetty (new 2025-2030 LTS)

### Testing

- Make sure that for all maps, all vehicles, a simple takeoff example works with up to 3 vehicles



-->
