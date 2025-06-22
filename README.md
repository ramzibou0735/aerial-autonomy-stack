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
