# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

1. **Develop** drone autonomy with ROS2
2. **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
3. **Deploy** in real drones with NVIDIA Orin/JetPack

> For the motivation behind AAS and how it compares to similar projects, read [`RATIONALE.md`](/docs/RATIONALE.md)

## Feature Highlights

- Support for multiple **quadrotors and VTOLs** based on **PX4 or ArduPilot**
- **ROS2**-based autopilot interfaces (*via* XRCE-DDS and MAVROS)
- Support for **YOLOv8** (with ONNX GPU Runtimes) and **Lidar Odometry** (with [KISS-ICP](https://github.com/PRBonn/kiss-icp))
- **Dockerized simulation** based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)

<details>
<summary><b>Additional Features:</b> <i>(expand)</i></summary>

> - **3D worlds** for [PX4](https://docs.px4.io/main/en/simulation/#sitl-simulation-environment) and [ArduPilot](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture) software-in-the-loop (SITL) simulation
> - **Steppable simulation** interface for reinforcement learning 
> - [Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) inter-vehicle ROS2 bridge
> - Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode (e.g. CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) 

</details>

<details>
<summary>AAS leverages the following frameworks: <i>(expand)</i></summary>

> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.15*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVROS](https://github.com/mavlink/mavros/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 6/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 6/2025)

</details>

<!-- TODO: add video of example startup/usage with API from git clone on -->

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [**`nvidia-driver-575`**](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 7/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060—**note that an NVIDIA GPU *is* required**
> 
> **To setup the requirements: (i) Ubuntu 22, Git LFS, (ii) NVIDIA driver, (iii) Docker Engine, (iv) NVIDIA Container Toolkit, and (v) NVIDIA NGC API Key, read [`PREINSTALL.md`](/docs/PREINSTALL.md)**

```sh
# Clone this repo
mkdir -p ~/git
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git ~/git/aerial-autonomy-stack
cd ~/git/aerial-autonomy-stack
```

### Option 1: Build the Docker Images

> [!WARNING]
> Building from scratch requires a stable internet connection, `Ctrl + c` and restart if needed 

```sh
docker build -t simulation-image -f docker/Dockerfile.simulation . # The first build takes ~15' and creates a 19GB image (8GB for ros-humble-desktop with nvidia runtime, 9GB for PX4 and ArduPilot SITL)
docker build -t aircraft-image -f docker/Dockerfile.aircraft . # The first build takes ~15' and creates a 16GB image (8GB for ros-humble-desktop with nvidia runtime, 7GB for YOLOv8, ONNX)
```

> These are development-friendly images with lots of tools and artifacts, trim as needed

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
chmod +x ./main_sim.sh
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=2 WORLD=swiss_town HEADLESS=false ./main_sim.sh # Read main_sim.sh for more options
# `Ctrl + b`, then `d` in each terminal once done
```

> Once "Ready to Fly", one can takeoff and control from QGroundControl's ["Fly View"](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html)

![worlds](https://github.com/user-attachments/assets/45a2f2ad-cc31-4d71-aa2e-4fe542a59a77)

Available `WORLD`s:
- `apple_orchard`, a GIS world created using [BlenderGIS](https://github.com/domlysz/BlenderGIS)
- `impalpable_greyness`, (default) an empty world with simple shapes
- `shibuya_crossing`, a 3D world adapted from [cgtrader](https://www.cgtrader.com/)
- `swiss_town`, a photogrammetry world courtesy of [Pix4D / pix4d.com](https://support.pix4d.com/hc/en-us/articles/360000235126)

To advance the simulation in **discrete time steps**, e.g. 1s, from a terminal on the host, run:

```sh
docker exec simulation-container bash -c "gz service -s /world/\$WORLD/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'multi_step: 250, pause: true'" # Adjust multi_step based on the value of max_step_size in the world's .sdf 
```

> [!TIP]
> <details>
> <summary>Tmux and Docker Shortcuts <i>(expand)</i></summary>
> 
> - Move between Tmux windows with `Ctrl + b`, then `n`, `p`
> - Move between Tmux panes with `Ctrl + b`, then `arrow keys`
> - Enter copy mode to scroll back with `Ctrl + [`, then `arrow keys`, exit with `q`
> - Detach Tmux with `Ctrl + b`, then `d`
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

### Run an Example

```sh
cd ~/git/aerial-autonomy-stack/
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=1 ./main_sim.sh
# In aircraft 1's terminal
./skibidi.sh
```

<!-- TODO: add video of the skibidi example -->

---

## Part 3: Development with AAS

```sh
cd ~/git/aerial-autonomy-stack/
chmod +x ./main_sim.sh
MODE=dev ./main_sim.sh # Images are pre-built but the ros2_ws/src/ folders are mounted from the host
```

*On the host*, edit the source of `~/git/aerial-autonomy-stack/aircraft_ws/src` and `~/git/aerial-autonomy-stack/simulation_ws/src`: it will be reflected in the two running containers

*In each of the two terminals/containers* created by `main_sim.sh`, re-build the workspaces

```sh
# In the simulation and aircraft 1 terminals
cd /ros2_ws
colcon build --symlink-install # rosdep update, rosdep install, if necessary
```

Start the simulation to check the changes

```sh
# In the simulation terminal
tmuxinator start -p /simulation_resources/simulation.yml.erb
# In aircraft 1's terminal
tmuxinator start -p /aircraft_resources/aircraft.yml.erb
```

Detach tmuxinator with `Ctrl + d` and kill the sessions

```sh
# In the simulation terminal
tmux kill-session -t simulation_tmuxinator && pkill -f gz
# In aircraft 1's terminal
tmux kill-session -t aircraft_tmuxinator
```

Repeat as necessary, finally commit the changes from the repository on the host computer

> [!NOTE]
> `aircraft_resources/` and `simulation_resources/` are also mounted but certain changes, e.g. in PX4's ROMFS, require compilation steps more easily achieved by re-building the Dockerfiles (see ["Part 1"](#option-1-build-the-docker-images))

---

## Part 4: Deployment of AAS

> [!IMPORTANT]
> These instructions are tested on a [Holybro Jetson Baseboard](https://holybro.com/products/pixhawk-jetson-baseboard) kit that includes (i) a Pixhawk 6X autopilot and (ii) an NVIDIA Orin NX 16GB computer connected via both serial and ethernet
> 
> **To setup (i) PX4's DDS UDP client, (ii) ArduPilot serial MAVLink bridge, (iii) JetPack 6, (iv) Docker Engine, (v) NVIDIA Container Toolkit, and (vi) NVIDIA NGC API Key on Orin, read [`AVIONICS.md`](/docs/AVIONICS.md)**
>
> The Holybro Jetson Baseboard comes with an (i) integrated 4-way (Orin, 6X, RJ-45, JST) Ethernet switch and (ii) two JST USB 2.0 that can be connected to ASIX Ethernet adapters to create additional network interfaces
> 
> Make sure to configure Orin, 6X's XRCE-DDS, IP radio, Zenoh, etc. consistently with your network setup; the camera acquisition pipeline should be setup in `yolo_inference_node.py`, the lidar should publish on topic `/lidar_points` for KISS-ICP (if necessary, discuss in the [Issues](https://github.com/JacopoPan/aerial-autonomy-stack/issues))

On Jetson Orin NX
```sh
mkdir -p ~/git
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git ~/git/aerial-autonomy-stack
cd ~/git/aerial-autonomy-stack

docker build -t aircraft-image -f docker/Dockerfile.aircraft . # The first build takes ~1h (mostly to build onnxruntime-gpu from source) and creates an 18GB image
# Alternatively
docker pull jacopopan/aircraft-image:jetson # TODO

chmod +x ./main_deploy.sh
DRONE_TYPE=quad AUTOPILOT=px4 DRONE_ID=1 CAMERA=true LIDAR=false ./main_deploy.sh
docker exec -it aircraft-container tmux attach
```

---
> You've done a man's job, sir. I guess you're through, huh?

<!-- 



## TODOs

- Implement ardupilot/mavros interface

#### PX4 interface
- replace "std::unique_lock" with "std::shared_lock" in read-only/non-writing threads/callbacks of the px4 interface
- check px4 interface action cancellations (check the sleep)
- QGC reports the px4 interface landing as a takeoff mode (?)
##### Offboard
- change mode is not working correctly (published in 2 blocks but not effective, command temporarily rejected))
- set default offboard duration consisrently with example vtol maneuvers (e.g. 3s less steep or no thrust dive)
##### Quad
- px4 quad cannot exit from orbit/only accepts orbit mode?
- change altitude for quad stops the current repositions (makes sense)
- change speed for quad only affects the next reposition and not orbit (movement and orbit speeds are param dependent)

- Make sure that for all maps, all vehicles, a simple autonomous takeoff + loiter + landing example works with up to 3 vehicles

### Known Issues

- Adjust orientation of the lidar and frame of the lidar odometry for VTOLs
- In yolo_inference_node.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumbent OpenCV)
- Should add gz::sim::systems::LiftDrag to multicopter for fast, high tilt flight



-->
