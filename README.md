# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:
- **Develop** drone autonomy using on ROS2
- **Simulate** perception and control in software-in-the-loop,using PX4/ArduPilot and YOLOv8
- **Deploy** in real drones based on NVIDIA JetPack

The stack is developed and tested on Ubuntu 22.04 (penultimate LTS, ESM 4/2032) with NVIDIA driver 570 (...) and Docker Engine v28 (latest stable release as of 6/2025)

It leverages the following frameworks:
- ROS2 Humble (LTS, EOL 5/2027)
- Gazebo Sim Harmonic (LTS, EOL 9/2028)
- PX4 1.15 (latest stable release as of 6/2025) interfaced *via* XRCE DDS (...)
- ArduPilot 4.6 (latest stable release as of 6/2025) interfaced *via* MAVSDK (...)
- YOLOv8 (tbd) and ONNX (tbd)
- L4T 36 (Ubuntu 22-based)/JetPack 6 (as deployment target, latest major release as of 6/2025)

https://github.com/nathanbowness/UAV-Object-Tracking
