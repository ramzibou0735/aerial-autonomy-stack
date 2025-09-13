# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

1. **Develop** end-to-end drone autonomy with ROS2
2. **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
3. **Deploy** in real drones with NVIDIA Orin/JetPack

> For the motivation behind AAS and how it compares to similar projects, read [`RATIONALE.md`](/supplementary/RATIONALE.md)

https://github.com/user-attachments/assets/0c60afc0-22bf-4ea0-b367-8691ecf6a3e7

## Feature Highlights

- Support for multiple **quadrotors and VTOLs** based on either **PX4 or ArduPilot**
- **ROS2**-based autopilot interfaces (*via* XRCE-DDS and MAVROS)
- Support for **YOLOv8** (with ONNX GPU Runtimes) and **LiDAR Odometry** (with [KISS-ICP](https://github.com/PRBonn/kiss-icp))
- **Dockerized simulation** based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
- **3D worlds** for [PX4](https://docs.px4.io/main/en/simulation/#sitl-simulation-environment) and [ArduPilot](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture) software-in-the-loop (SITL) simulation
- [Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) inter-vehicle ROS2 bridge
- Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode (e.g. CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) 
- Support for [ArduPilot Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) mode (i.e. `setpoint_velocity`, `setpoint_accel` references)
- Logs analysis with [`flight_review`](https://github.com/PX4/flight_review) (`.ulg`), MAVExplorer (`.bin`), and [PlotJuggler](https://github.com/facontidavide/PlotJuggler) (`rosbag`)
- **Steppable simulation** interface for reinforcement learning
- Support for Gazebo's [**`WindEffects`**](https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/worlds/wind.sdf) (except PX4 VTOL)

<details>
<summary>AAS leverages the following frameworks: <i>(expand)</i></summary>

> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.16*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVROS](https://github.com/mavlink/mavros/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 8/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 8/2025)

</details>

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [**`nvidia-driver-575`**](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 7/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060—**note that an NVIDIA GPU *is* required**
> 
> **To setup the requirements: (i) Ubuntu 22, Git LFS, (ii) NVIDIA driver, (iii) Docker Engine, (iv) NVIDIA Container Toolkit, and (v) NVIDIA NGC API Key, read [`PREINSTALL.md`](/supplementary/PREINSTALL.md) (for Windows, read [`WSL.md`](/supplementary/WSL.md))**

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
# Start a simulation (note: ArduPilot STIL takes ~40s to be ready to arm)
cd ~/git/aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=1 WORLD=swiss_town ./sim_run.sh # Check the script for more options
```

> On a low-mid range laptop—i7-11 with 16GB RAM and RTX3060—AAS simulates 3 PX4 quads with camera and LiDAR in the default world at 99% of the wall-clock (ArduPilot faster physics updates and more complex worlds can have higher computational demands)
>
> Once "Ready to Fly", one can takeoff and control from QGroundControl's ["Fly View"](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html)

> [!NOTE]
> <details>
> <summary>Simulation for Windows-based build (WSL): <i>(expand)</i></summary>
> On a native Linux desktop, the simulation scripts use **`gnome-terminal`** to spawn multiple windows (one for the simulator and one for each aircraft).  
> For WSL2, we use **`xterm`**, a lightweight X11 terminal emulator, where graphical applications are forwarded to Windows through an external X server (such as VcXsrv or Xming on Windows).  
> 
> To start the simulation in WSL2, use the `sim_run.sh` script located in `/scripts/xterm`:
> ```sh
> cd ~/git/aerial-autonomy-stack/scripts/xterm
> DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=2 WORLD=swiss_town ./sim_run.sh
> ```
</details>

![worlds](https://github.com/user-attachments/assets/45a2f2ad-cc31-4d71-aa2e-4fe542a59a77)

Available `WORLD`s:
- `apple_orchard`, a GIS world created using [BlenderGIS](https://github.com/domlysz/BlenderGIS)
- `impalpable_greyness`, (default) an empty world with simple shapes
- `shibuya_crossing`, a 3D world adapted from [cgtrader](https://www.cgtrader.com/)
- `swiss_town`, a photogrammetry world courtesy of [Pix4D / pix4d.com](https://support.pix4d.com/hc/en-us/articles/360000235126)

To advance the simulation in **discrete time steps**, e.g. 1s, from a terminal on the host, run:

```sh
docker exec simulation-container bash -c "gz service -s /world/\$WORLD/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'multi_step: 250, pause: true'" # Adjust multi_step based on the value of max_step_size in the world's .sdf (defaults: 250 for PX4, 1000 for ArduPilot)
```

To add or disable a **wind field**, from a terminal on the host, run:

```sh
docker exec simulation-container bash -c "gz topic -t /world/\$WORLD/wind/ -m gz.msgs.Wind  -p 'linear_velocity: {x: 0.0 y: 3.0}, enable_wind: true'" # Positive X blows from the West, positive Y blows from the South

docker exec simulation-container bash -c "gz topic -t /world/\$WORLD/wind/ -m gz.msgs.Wind  -p 'enable_wind: false'" # Disable WindEffects
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
> </details>

### Fly a Mission

```sh
cd ~/git/aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 ./sim_run.sh # Also try AUTOPILOT=ardupilot, or NUM_QUADS=0 NUM_VTOLS=1

# In aircraft 1's terminal
ros2 run mission mission --conops yalla --ros-args -r __ns:=/Drone$DRONE_ID -p use_sim_time:=true # This mission is a simple takeoff, followed by an orbit, and landing for any vehicle

# Finally, in the simulation's terminal
/simulation_resources/patches/plot_logs.sh # Analyze the flight logs
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

> [!NOTE]
> Project Structure
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
> │   ├── docker
> │   │   ├── Dockerfile.aircraft     # Docker image for aircraft simulation and deployment
> │   │   └── Dockerfile.simulation   # Docker image for Gazebo and SITL simulation
> │   │
> │   ├── deploy_build.sh             # Build `Dockerfile.aircraft` for arm64/Orin
> │   ├── deploy_run.sh               # Start the aircraft docker on arm64/Orin
> │   │
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

- https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera
- https://github.com/Livox-SDK/livox_ros_driver2
- Replace _create_ardupilot_world.sh and _create_ardupilot_models.sh with ruby/erb 
- Simplify ArdupilotInterface
- Add state estimation package/node
- Add bounding-box-based Offboard
- Create smaller docker images to use GitHub Actions
- For PX4 quad max tilt maneuver, zero the anti-windup gain: const float arw_gain = 2.f / _gain_vel_p(0);
- ????
- Profit

### Known Issues

- QGC is started with a virtual joystick (with low throttle if using only VTOLs and centered throttle if there are quads), this is reflective of real-life but note that this counts as "RC loss" when switching focus from one autopilot instance to another
- ArduPilot CIRCLE mode for quads require to explicitly center the virtual throttle with 'rc 3 1500' to keep altitude
- Gazebo WindEffects plugin is disabled for PX4 standard_vtol
- Command 178 MAV_CMD_DO_CHANGE_SPEED is accepted but not effective in changing speed for ArduPilot VTOL
- ArduPilot SITL for Iris uses option -f that also sets "external": True, this is not the case for the Alti Transition from ArduPilot/SITL_Models 
- In ArdupilotInterface's action callbacks, std::shared_lock<std::shared_mutex> lock(node_data_mutex_); could be used on the reads of lat_, lon_, alt_
- In yolo_inference_node.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumvent OpenCV)
- QGC does not save roll and pitch in the telemetry bar for PX4 VTOLs (MAV_TYPE 22)



-->
