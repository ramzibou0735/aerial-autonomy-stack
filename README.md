# aerial-autonomy-stack

Aerial autonomy stack (AAS) is a software stack to:
- Develop drone autonomy (based on ROS2)
- Simulate in SITL (using PX4 and Ardupilot)
- Deploy in real drones (on NVIDIA Jetson with L4T)

The stack is developed and tested on Ubuntu 22.04 (penultimate LTS, ESM 4/2032)

It combines the following frameworks:
- Docker Engine v28/Docker Compose v2.37 (latest stable releases as of 6/2025)
- ROS2 Humble (LTS, EOL 5/2027)
- Gazebo Sim Harmonic (LTS, EOL 9/2028)
- PX4 1.15 (latest stable release as of 6/2025)
- Ardupilot 4.6 (latest stable release as of 6/2025)
- L4T 36 (Ubuntu 22-based)/JetPack 6 (as targed deployment OS, latest major release as of 6/2025)

It leverages the following tools:
- XRCE DDS (to interface PX4 and ROS2)
- Pymavlink (to interface with Ardupilot)
- QGroundControl (for ground control)
- Tmuxinator (for workflow automation)
- YOLOv8 (for object detection)
- BlenderGIS (for world creation)

It supports the following aircraft:
- quadcopters
- VTOLs

It supports the following sensors:
- RGB/EO fixed-mount camera
- 360deg lidar
- 3D radar

## External Tracking Examples

| X500 Quadcopter                        |  Standard VTOL                         |
|:--------------------------------------:|:--------------------------------------:|
| [`x500_ext_px4_navigator.sh`](link)    | [`vtol_ext_px4_navigator.sh`](link)    |
| [`x500_ext_ardupilot_guided.sh`](link) | [`vtol_ext_ardupilot_guided.sh`](link) |
| [`x500_ext_px4_ctbr.sh`](link)         |                                        |

## On-board Tracking Examples

| X500 Quadcopter                        |  Standard VTOL                         |
|:--------------------------------------:|:--------------------------------------:|
| [`x500_rgb_px4_navigator.sh`](link)    | [`vtol_rgb_px4_navigator.sh`](link)    |
| [`x500_rgb_ardupilot_guided.sh`](link) | [`vtol_rgb_ardupilot_guided.sh`](link) |
| [`x500_rgb_px4_ctbr.sh`](link)         |                                        |

## Autopilot Interfaces

PX4: leverage XRCE DDS and ROS2 C++ for Offboard mode and/or navigator-level vehicle commands

ArduPilot: ROS2 Python wrapping of vehicle commands in "Guided Mode" (could include local frame velocity) with https://github.com/ArduPilot/pymavlink
- Alternatively, use https://github.com/ArduPilot/ardupilot-mavsdk
- https://ardupilot.org/dev/docs/mavlink-commands.html

## Drone Models

- Start from PX4 X500, Standard VTOL .sdf for `gz sim`
- Adapt ArduPilot models from https://ardupilot.org/dev/docs/sitl-with-gazebo.html (do NOT listen to ChatGPT who thinks ArduPilot is stuck on Gazebo Classic and hallucinates repos)
- Add `camera` and `gpu_lidar`
- For YOLOv8, radard driver interfaces, see https://github.com/nathanbowness/UAV-Object-Tracking

## JSBSim

- PX4 SITL support for JSBSim is legacy https://docs.px4.io/main/en/sim_jsbsim/index.html
- ArduPilot's is not very documented https://ardupilot.org/dev/docs/sitl-with-jsbsim.html

## Future Work

- Basic fighter maneuvers (BFM) for VTOLs *via* PX4 Offboard Mode in CTBR
