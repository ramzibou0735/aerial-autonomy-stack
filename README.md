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
> - Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode (including CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) 

</details>

<details>
<summary>AAS leverages the following frameworks: <i>(expand)</i></summary>

> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.15*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVROS](https://github.com/mavlink/mavros/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 6/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 6/2025)

</details>

<!-- TODO: add video -->

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [**`nvidia-driver-575`**](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 7/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060
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
docker build -t simulation-image -f docker/Dockerfile.simulation . # The first build takes ~15' and creates a 19GB image (6GB for ros-humble-desktop, 9GB for PX4 and ArduPilot SITL)
docker build -t aircraft-image -f docker/Dockerfile.aircraft . # Having built Dockerfile.simulation, the first build takes ~15' and creates a 15GB image (6GB for ros-humble-desktop, 7GB for YOLOv8, ONNX)
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

<!-- TODO: add video -->

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
> **To setup PX4 parameters and DDS client, read [`PX4_SETUP.md`](/docs/PX4_SETUP.md)**
> 
> **To setup ArduPilot MAVLink interface, read [`ARDUPILOT_SETUP.md`](/docs/ARDUPILOT_SETUP.md)**

WIP
<!-- 
```sh
cd ~/git/aerial-autonomy-stack/
chmod +x ./main_deploy.sh
DRONE_TYPE=quad AUTOPILOT=px4 DRONE_ID=1 CAMERA=true LIDAR=false  ./main_deploy.sh
docker exec -it aircraft-container tmux attach
```
-->

---
> You've done a man's job, sir. I guess you're through, huh?

<!-- 



## TODOs

- replace "std::unique_lock" with "std::shared_lock" in read-only/non-writing threads/callbacks
- check action cancellations

### Known Issues

- QGC reports the quad landing as a takeoff mode
- mavros commands require multiple resend
```sh
ros2 topic pub --once /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 10.0, y: 0.0, z: 5.0}}}'
```
- Revise orientation of the lidar and frame of the lidar odometry for VTOLs
- In yolo.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumbent OpenCV)

### Testing

- Make sure that for all maps, all vehicles, a simple takeoff example works with up to 3 vehicles

### Model Resources

- ArduPilot SITL models: https://github.com/ArduPilot/SITL_Models



-->
