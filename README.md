# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

1. **Develop** end-to-end drone autonomy with ROS2
2. **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
3. **Deploy** in real drones with NVIDIA Orin/JetPack

> For the motivation behind AAS and how it compares to similar projects, read [`RATIONALE.md`](/supplementary/RATIONALE.md)

## Feature Highlights

- Support for multiple **quadrotors and VTOLs** based on either **PX4 or ArduPilot**
- **ROS2**-based autopilot interfaces (*via* XRCE-DDS and MAVROS)
- Support for **YOLOv8** (with ONNX GPU Runtimes) and **LiDAR Odometry** (with [KISS-ICP](https://github.com/PRBonn/kiss-icp))
- **Dockerized simulation** based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)

<details>
<summary><b>Additional Features:</b> <i>(expand)</i></summary>

> - **3D worlds** for [PX4](https://docs.px4.io/main/en/simulation/#sitl-simulation-environment) and [ArduPilot](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture) software-in-the-loop (SITL) simulation
> - [Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) inter-vehicle ROS2 bridge
> - Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode (e.g. CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) and [ArduPilot Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) mode (i.e. `setpoint_velocity`, `setpoint_accel` references)
> - **Steppable simulation** interface for reinforcement learning 

</details>

https://github.com/user-attachments/assets/0a601d12-873c-4a7a-873d-e4359ccb5c44

<details>
<summary>AAS leverages the following frameworks: <i>(expand)</i></summary>

> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.16*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVROS](https://github.com/mavlink/mavros/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 8/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 8/2025)

</details>

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [**`nvidia-driver-575`**](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 7/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060—**note that an NVIDIA GPU *is* required**
> 
> **To setup the requirements: (i) Ubuntu 22, Git LFS, (ii) NVIDIA driver, (iii) Docker Engine, (iv) NVIDIA Container Toolkit, and (v) NVIDIA NGC API Key, read [`PREINSTALL.md`](/supplementary/PREINSTALL.md)**

```sh
# Clone this repo
mkdir -p ~/git
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git ~/git/aerial-autonomy-stack
cd ~/git/aerial-autonomy-stack
```

### Build the Docker Images

> [!WARNING]
> The build script creates two ~20GB images (including lots of tools and artifacts for development)
> 
> Building from scratch requires a good/stable internet connection (`Ctrl + c` and restart if necessary)

```sh
# Clone external repos (in github_clones/) and build the Docker images
cd ~/git/aerial-autonomy-stack/scripts
./sim_build.sh # The first build takes ~25', subsequent ones take seconds to minutes
```

---

## Part 2: Simulation and Development with AAS

```sh
# Run a simulation
cd ~/git/aerial-autonomy-stack/scripts
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=2 WORLD=swiss_town ./sim_run.sh # Check the script for more options
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
# Pause the simulation
docker exec simulation-container bash -c "gz service -s /world/\$WORLD/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'multi_step: 250, pause: true'" # Adjust multi_step based on the value of max_step_size in the world's .sdf 
```

> [!TIP]
> <details>
> <summary>Tmux and Docker Shortcuts <i>(expand)</i></summary>
> 
> - Move between Tmux windows with `Ctrl + b`, then `n`, `p`
> - Move between Tmux panes with `Ctrl + b`, then `arrow keys`
> - Enter copy mode to scroll back with `Ctrl + [`, then `arrow keys`, exit with `q`
> - Split a Tmux window with `Ctrl + b`, then `"` (horizontal) or `%` (vertical)
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
> docker builder prune # Clear the cache system wide
> ```
> 
> </details>

### Fly a Mission

```sh
cd ~/git/aerial-autonomy-stack/scripts
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=1 ./sim_run.sh
# In aircraft 1's terminal
ros2 run mission mission --conops yalla --ros-args -r __ns:=/Drone$DRONE_ID -p use_sim_time:=true
# This mission is a simple takeoff, followed by an orbit, and landing
# Works for all combinations of AUTOPILOT=px4 or ardupilot, DRONE_TYPE=quad or vtol
```

### Command Line Interface

Read the banner comment in the `autopilot_interface` headers for command line examples (takeoff, orbit, reposition, offboard, land):

- [`ardupilot_interface.hpp`](/aircraft/aircraft_ws/src/autopilot_interface/src/ardupilot_interface.hpp): ArduPilot actions and services
- [`px4_interface.hpp`](/aircraft/aircraft_ws/src/autopilot_interface/src/px4_interface.hpp): PX4 actions and services

Once flown from CLI, implemented your mission in [`MissionNode.conops_callback()`](/aircraft/aircraft_ws/src/mission/mission/mission_node.py)


### Development

Launching the `sim_run.sh` script with `MODE=dev`, does **not** start the simulation and mounts folders `simulation_resources`, `aircraft_resources`, and `ros2_ws/src` as volumes to more easily track, commit, push changes while building and testing them within the containers

```sh
# Develop within live containers
cd ~/git/aerial-autonomy-stack/scripts
MODE=dev ./sim_run.sh # Images are pre-built but the ros2_ws/src/ and *_resources/ folders are mounted from the host
```

> [!TIP]
> <details>
> <summary>Package Structure <i>(expand)</i></summary>
> 
> ```sh
> aerial-autonomy-stack
> │
> ├── aircraft
> │   ├── aircraft_ws
> │   │   └── src
> │   │       ├── autopilot_interface # Ardupilot/PX4 high-level actions (Takeoff, Orbit, Offboard, Land)
> │   │       ├── mission             # Orchestrator of the actions in `autopilot_interface` 
> │   │       ├── offboard_control    # Low-level references for the Offboard action in `autopilot_interface` 
> │   │       ├── state_sharing       # Publisher of the `/state_sharing_drone_N` topic broadcasted by Zenoh
> │   │       └── yolo_inference      # GStreamer video acquisition and publisher of YOLO bounding boxes
> │   │
> │   └── aircraft.yml.erb            # Aircraft docker tmux entrypoint
> │
> ├── scripts
> │   ├── deploy_build.sh             # Build `Dockerfile.aircraft` for arm64/Orin
> │   ├── deploy_run.sh               # Start the aircraft docker on arm64/Orin
> │   ├── docker
> │   │   ├── Dockerfile.aircraft     # Docker image for aircraft simulation and deployment
> │   │   └── Dockerfile.simulation   # Docker image for Gazebo and SITL simulation
> │   ├── sim_build.sh                # Build both dockerfiles for amd64/simulation
> │   └── sim_run.sh                  # Start the simulation
> │
> └── simulation
>     ├── simulation_resources
>     │   ├── aircraft_models
>     │   │   ├── alti_transition_quad # ArduPilot VTOL
>     │   │   ├── iris_with_ardupilot  # ArduPilot quad
>     │   │   ├── sensor_camera
>     │   │   ├── sensor_lidar
>     │   │   ├── standard_vtol        # PX4 VTOL
>     │   │   └── x500                 # PX4 quad
>     │   └── simulation_worlds
>     │       ├── apple_orchard.sdf
>     │       ├── impalpable_greyness.sdf
>     │       ├── shibuya_crossing.sdf
>     │       └── swiss_town.sdf
>     │
>     ├── simulation_ws
>     │   └── src
>     │       └── ground_system        # Publisher of topic `/tracks` broadcasted by Zenoh
>     │
>     └── simulation.yml.erb           # Simulation docker tmux entrypoint
> ```
> </details>

---

## Part 3: Deployment of AAS

> [!IMPORTANT]
> These instructions are tested on a [Holybro Jetson Baseboard](https://holybro.com/products/pixhawk-jetson-baseboard) kit that includes (i) a Pixhawk 6X autopilot and (ii) an NVIDIA Orin NX 16GB computer connected via both serial and ethernet
> 
> **To setup (i) PX4's DDS UDP client, (ii) ArduPilot serial MAVLink bridge, (iii) JetPack 6, (iv) Docker Engine, (v) NVIDIA Container Toolkit, and (vi) NVIDIA NGC API Key on Orin, read [`AVIONICS.md`](/supplementary/AVIONICS.md)**
>
> The Holybro Jetson Baseboard comes with an (i) integrated 4-way (Orin, 6X, RJ-45, JST) Ethernet switch and (ii) two JST USB 2.0 that can be connected to ASIX Ethernet adapters to create additional network interfaces
> 
> Make sure to configure Orin, 6X's XRCE-DDS, IP radio, Zenoh, etc. consistently with your network setup; the camera acquisition pipeline should be setup in `yolo_inference_node.py`, the LiDAR should publish on topic `/lidar_points` for KISS-ICP (if necessary, discuss in the [Issues](https://github.com/JacopoPan/aerial-autonomy-stack/issues))


```sh
# On Jetson Orin NX, build for arm64 with TensorRT support
mkdir -p ~/git
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git ~/git/aerial-autonomy-stack
cd ~/git/aerial-autonomy-stack/scripts
./deploy_build.sh # The first build takes ~1h (mostly to build onnxruntime-gpu from source)
```

```sh
# On Jetson Orin NX, start and attach the aerial-autonomy-stack (e.g., from ssh)
DRONE_TYPE=quad AUTOPILOT=px4 DRONE_ID=1 CAMERA=true LIDAR=false ./deploy_run.sh
docker exec -it aircraft-container tmux attach
```

---
> You've done a man's job, sir. I guess you're through, huh?

<!-- 

## TODOs

- Minor video edits: "via" Zenoh, Take-off
- Upgrade to nvidia-driver-580
- Remove PX4 MPC acceleration limit
- Create GitHub action builds
- Simplify ArdupilotInterface
- ????
- Profit

### Known Issues

- Ground tracks in topic /tracks are occasionally not published
- In ArdupilotInterface's action callbacks, std::shared_lock<std::shared_mutex> lock(node_data_mutex_); could be used on the reads of lat_, lon_, alt_
- Command 178 MAV_CMD_DO_CHANGE_SPEED is accepted but not effective in changing speed for ArduPilot VTOL
- ArduPilot SITL for Iris uses option -f that also sets "external": True, this is not the case for the Alti Transition from ArduPilot/SITL_Models 
- Must adjust orientation of the LiDAR and frame of the LiDAR odometry for VTOLs
- In yolo_inference_node.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumbent OpenCV)
- ROS2 action cancellation from CLI does not work (File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 723, in wait_for_ready_callbacks - return next(self._cb_iter) - ValueError: generator already executing), use cancellable_action.py instead
- Cannot use **/.git in .dockerignore because PX4 and ArduPilot use it in their build
- PX4 messages 1.16 have VehicleStatus on MESSAGE_VERSION = 1, topic fmu/out/vehicle_status_v1
- QGC does not save roll and pitch in the telemetry bar for PX4 VTOLs (MAV_TYPE 22)
- QGC is started with a virtual joystick (with low throttle for VTOLs and centered throttle for quads), this is reflective of real-life but note that this counts as "RC loss" when switching focus from one autopilot instance to another



-->
