# aerial-autonomy-stack

Aerial autonomy stack (AAS) is a software stack to:
- Develop drone autonomy (based on ROS2)
- Simulate in SITL (using PX4 and Ardupilot)
- Deploy in real drones (on NVIDIA Jetson with L4T)

It is build around the following tools:
- Docker
- ROS2
- Gazebo Sim
- PX4
- Ardupilot
- XRCE DDS (to interface PX4 and ROS2)
- Pymavlink (to interface with Ardupilot)
- QGroundControl (for ground control)
- L4T/JetPack (as targed deployment OS)
- Tmuxinator (for workflow automation)
- YOLO (for object detection)
- BlenderGIS (for world creation)

It supports the following aircraft:
- quadcopters
- VTOLs

It supports the following sensors:
- RGB/EO fixed-mount camera
- 360deg lidar
- 3D radar
