# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

1. **Develop** end-to-end drone autonomy with ROS2
2. **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
3. **Deploy** in real drones with NVIDIA Orin/JetPack

> For the motivation behind AAS and how it compares to similar projects, read [`RATIONALE.md`](/supplementary/RATIONALE.md)

## Feature Highlights

- Support for multiple **quadrotors and VTOLs** based on either **PX4 or ArduPilot**
- **ROS2**-based autopilot interfaces (*via* XRCE-DDS and MAVROS)
- Support for **YOLOv8** (with ONNX GPU Runtimes) and **Lidar Odometry** (with [KISS-ICP](https://github.com/PRBonn/kiss-icp))
- **Dockerized simulation** based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)

<details>
<summary><b>Additional Features:</b> <i>(expand)</i></summary>

> - **3D worlds** for [PX4](https://docs.px4.io/main/en/simulation/#sitl-simulation-environment) and [ArduPilot](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture) software-in-the-loop (SITL) simulation
> - **Steppable simulation** interface for reinforcement learning 
> - [Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) inter-vehicle ROS2 bridge
> - Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode (e.g. CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) and [ArduPilot Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) mode (including `setpoint_velocity`, `setpoint_accel` references)

</details>

<details>
<summary>AAS leverages the following frameworks: <i>(expand)</i></summary>

> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.16*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVROS](https://github.com/mavlink/mavros/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 8/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 8/2025)

</details>

<!-- TODO: add video of example startup/usage with API from git clone on -->

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

### Run an Example

```sh
# Simple vision-based guidance
cd ~/git/aerial-autonomy-stack/scripts
DRONE_TYPE=quad AUTOPILOT=px4 NUM_DRONES=1 ./sim_run.sh
# In aircraft 1's terminal
./skibidi.sh
```

<!-- TODO: add video of the skibidi example -->

### Development

Launching the `sim_run.sh` script with `MODE=dev`, does **not** start the simulation and mounts folders `simulation_resources`, `aircraft_resources`, and `ros2_ws/src` as volumes to more easily track, commit, push changes while building and testing them within the containers

```sh
# Develop within live containers
cd ~/git/aerial-autonomy-stack/scripts
MODE=dev ./sim_run.sh # Images are pre-built but the ros2_ws/src/ and *_resources/ folders are mounted from the host
```

---

## Part 3: Deployment of AAS

> [!IMPORTANT]
> These instructions are tested on a [Holybro Jetson Baseboard](https://holybro.com/products/pixhawk-jetson-baseboard) kit that includes (i) a Pixhawk 6X autopilot and (ii) an NVIDIA Orin NX 16GB computer connected via both serial and ethernet
> 
> **To setup (i) PX4's DDS UDP client, (ii) ArduPilot serial MAVLink bridge, (iii) JetPack 6, (iv) Docker Engine, (v) NVIDIA Container Toolkit, and (vi) NVIDIA NGC API Key on Orin, read [`AVIONICS.md`](/supplementary/AVIONICS.md)**
>
> The Holybro Jetson Baseboard comes with an (i) integrated 4-way (Orin, 6X, RJ-45, JST) Ethernet switch and (ii) two JST USB 2.0 that can be connected to ASIX Ethernet adapters to create additional network interfaces
> 
> Make sure to configure Orin, 6X's XRCE-DDS, IP radio, Zenoh, etc. consistently with your network setup; the camera acquisition pipeline should be setup in `yolo_inference_node.py`, the lidar should publish on topic `/lidar_points` for KISS-ICP (if necessary, discuss in the [Issues](https://github.com/JacopoPan/aerial-autonomy-stack/issues))


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

- Implement MAVROS subscribers to ardupilot_interface

/mavros/global_position/global
    NavSatFix
/mavros/local_position/odom 
    nav_msgs/msg/Odometry
    (alternatives /mavros/global_position/local nav_msgs/msg/Odometry AND /mavros/local_position/pose geometry_msgs/msg/PoseStamped)

/mavros/global_position/local
  nav_msgs/msg/Odometry

/mavros/local_position/velocity_body
    geometry_msgs/msg/TwistStamped
    (alternative /mavros/local_position/velocity_local geometry_msgs/msg/TwistStamped)

/mavros/vfr_hud
  mavros_msgs/msg/VfrHud

/mavros/home_position/home 
    mavros_msgs/msg/HomePosition

/mavros/state
    mavros_msgs/msg/State

finally ros2 service call /mavros/vehicle_info_get mavros_msgs/srv/VehicleInfoGet

----------------
alt_ = ???; // AMSL
    /mavros/vfr_hud 
    Type: mavros_msgs/msg/VfrHud

//  xy_valid_ = msg->xy_valid;
// z_valid_ = msg->z_valid;
// v_xy_valid_ = msg->v_xy_valid;
// v_z_valid_ = msg->v_z_valid;
// Position in local NED frame
x_ = msg->x; // N
y_= msg->y; // E
z_ = msg->z; // D
    /mavros/global_position/local
    nav_msgs/msg/Odometry
heading_ = msg->heading; // Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
    /mavros/global_position/compass_hdg 
    std_msgs/msg/Float64
    also in
    /mavros/vfr_hud 
    Type: mavros_msgs/msg/VfrHud
// Velocity in NED frame
vx_ = msg->vx;
vy_ = msg->vy;
vz_ = msg->vz;
    see /mavros/global_position/local
    nav_msgs/msg/Odometry ?
// Angular velocity in NED frame
// Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
// xy_global_ = msg->xy_global; // Validity of reference
// z_global_ = msg->z_global; // Validity of reference
ref_lat_ = msg->ref_lat;
ref_lon_ = msg->ref_lon;
ref_alt_ = msg->ref_alt; // AMSL
    /mavros/home_position/home 
    mavros_msgs/msg/HomePosition
    but altitude is ellipsoid

// pose_frame_ = msg->pose_frame; // 1:  NED earth-fixed frame, 2: FRD world-fixed frame, arbitrary heading
// velocity_frame_ = msg->velocity_frame; // 1:  NED earth-fixed frame, 2: FRD world-fixed frame, arbitrary heading, 3: FRD body-fixed frame
position_ = msg->position;
q_ = msg->q;
    /mavros/global_position/local
    nav_msgs/msg/Odometry

    but also (probably better)
    /mavros/local_position/odom 
    nav_msgs/msg/Odometry
    although the twist part is different (is is the lin/ang vel???)

    as well as (only position and quat)
    /mavros/local_position/pose
    geometry_msgs/msg/PoseStamped

velocity_ = msg->velocity;
angular_velocity_ = msg->angular_velocity;

    /mavros/local_position/velocity_body
    geometry_msgs/msg/TwistStamped
    (is this the same twist as local_position/odom ???)

    what is the difference with:
    /mavros/local_position/velocity_local


true_airspeed_m_s_ = msg->true_airspeed_m_s;
    /mavros/vfr_hud 
    Type: mavros_msgs/msg/VfrHud

command_ack_ = msg->command;
command_ack_result_ = msg->result;
command_ack_from_external_ = msg->from_external;
    mode from mavros_msgs/msg/State is probably more interesting

target_system_id_ = msg->system_id; // get target_system_id from PX4's MAV_SYS_ID once
    see ros2 service call /mavros/vehicle_info_get

arming_state_ = msg->arming_state; // DISARMED = 1, ARMED = 2
    /mavros/state
    mavros_msgs/msg/State
    (incldes mode too)

vehicle_type_ = msg->vehicle_type; // ROTARY_WING = 1, FIXED_WING = 2 (ROVER = 3)
is_vtol_ = msg->is_vtol; // bool
is_vtol_tailsitter_ = msg->is_vtol_tailsitter; // bool
in_transition_mode_ = msg->in_transition_mode; // bool
in_transition_to_fw_ = msg->in_transition_to_fw; // bool
    ros2 service call /mavros/vehicle_info_get mavros_msgs/srv/VehicleInfoGet
    requester: making request: mavros_msgs.srv.VehicleInfoGet_Request(sysid=0, compid=0, get_all=False)
    response:
    mavros_msgs.srv.VehicleInfoGet_Response(success=True, vehicles=[mavros_msgs.msg.VehicleInfo(header=std_msgs.msg.Header(
        stamp=builtin_interfaces.msg.Time(sec=2254, nanosec=211000000), frame_id=''), available_info=3, sysid=1, compid=1, autopilot=3, type=2, 
        system_status=4, base_mode=217, custom_mode=4, mode='GUIDED', mode_id=4, capabilities=64495, flight_sw_version=67502847,
        middleware_sw_version=0, os_sw_version=0, board_version=0, flight_custom_version='3939643464626531', vendor_id=0, product_id=0, uid=0)])

pre_flight_checks_pass_ = msg->pre_flight_checks_pass; // bool
    can use MAV_STATE in
    /mavros/state
    mavros_msgs/msg/State

----------------

- Remove set_altitude from PX4Interface
- Change Orbit to an action

- Implement ardupilot_interface services
- Implement ardupilot_interface actions

- Determine how to send rates, attitude, trajectory, velocity, acceleration references for Offboard/Guided modes

- Create and implement vision/control node

- Double check mutex and sleep use in px4_interface
- Make sure that for all maps, all vehicles, a simple autonomous takeoff + loiter + landing example works with up to 3 vehicles with sensors

### Known Issues

- Command 178 MAV_CMD_DO_CHANGE_SPEED is accepted but not effective in changing speed for VTOL
- ArduPilot SITL for Iris uses option -f that also sets "external": True, this is not the case for the Alti Transition from ArduPilot/SITL_Models 
- Must adjust orientation of the lidar and frame of the lidar odometry for VTOLs
- In yolo_inference_node.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumbent OpenCV)
- ROS2 action cancellation from CLI does not work (File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 723, in wait_for_ready_callbacks - return next(self._cb_iter) - ValueError: generator already executing), use cancellable_action.py instead
- Cannot use **/.git in .dockerignore because PX4 and ArduPilot use it in their build
- PX4 messages 1.16 have VehicleStatus on MESSAGE_VERSION = 1, topic fmu/out/vehicle_status_v1
- QGC does not save roll and pitch in the telemetry bar for PX4 VTOLs (MAV_TYPE 22)
- QGC is started with a virtual joystick (with low throttle for VTOLs and centered throttle for quads), this is reflective of real-life but note that this counts as "RC loss" when switching focus from one autopilot instance to another



-->
