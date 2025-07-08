# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:
- **Develop** drone autonomy with ROS2
- **Simulate** vision and control in software-in-the-loop, with YOLOv8 and PX4/ArduPilot
- **Deploy** in real drones with NVIDIA Orin/JetPack

> [!NOTE]
> For the motivation behind AAS (and how it compares to similar projects), read [`RATIONALE.md`](/docs/RATIONALE.md)

## Feature Highlights

- Support for **quadrotor and VTOL** aircraft based on **PX4 or ArduPilot**
- Support for **ROS2** with ROS2-based autopilot interfaces (*via* XRCE-DDS and MAVSDK)
- Support for **YOLOv8** and ONNX CPU, CUDA (on desktop/amd64), and TensorRT (on Orin/arm64) Runtime
- Photorealistic **software-in-the-loop simulation**
- Dockerized simulation based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)

**Additional Features**

- Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode in CTBR (`VehicleRatesSetpoint`) for agile, GNSS-denied flight
- Support for [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (Fast LiDAR-Inertial Odometry)
- Lightweight inter-drone serial communication for real-world deployment 

> [!NOTE]
> AAS leverages the following frameworks:
>
> [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.15*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVSDK](https://github.com/mavlink/mavsdk/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 6/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 6/2025)

---

## Part 1: Installation of AAS

> [!IMPORTANT]
> This stack is developed and tested using a [Ubuntu 22.04](https://ubuntu.com/about/release-cycle) host (penultimate LTS, ESM 4/2032) with [`nvidia-driver-570`](https://developer.nvidia.com/datacenter-driver-archive) and Docker Engine v28 (latest stable releases as of 6/2025) on an i9-13 with RTX3500 and an i7-11 with RTX3060 computers
> 
> To setup (i) Ubuntu 22, (ii) the NVIDIA driver, (iii) Docker Engine, and (iv) NVIDIA Container Toolkit, read [`PREINSTALL.md`](/docs/PREINSTALL.md)

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
# The first build takes ~15' and creates an 18GB image
docker build -t simulation-image -f Dockerfile.simulation . # (8GB for ros-humble-desktop, 9GB for PX4 and ArduPilot SITL)

# Having built Dockerfile.simulation, the first build takes ~15' and creates a 16GB image
docker build -t aircraft-image -f Dockerfile.aircraft . # (8GB for ros-humble-desktop, 7GB for YOLOv8, ONNX)
```

> [!TIP]
> These are development-friendly images with lots of tools and artifacts, trim if needed
> - Removing `ultralytics`, YOLOv8, and ONNX saves ~7GB from `aircraft-image`
> - Using only one between PX4 and ArduPilot saves ~4GB from `simulation-image`
> - Using `ros-humble-ros-base` instead of `ros-humble-desktop` saves ~1GB from from `simulation-image` and ~3GB from `aircraft-image`


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
DRONE_TYPE=vtol AUTOPILOT=ardupilot NUM_DRONES=1 ./main.sh
```

> [!TIP]
> Tmux shortcuts
> - Move between windows with `Ctrl + b`, then `n`, `p`
> - Move between panes with `Ctrl + b`, then `arrows`
> - Detach with `Ctrl + b`, then press `d`
> - Re-attach with `$ tmux attach-session -t simulation_tmuxinator`
> - Or kill with `$ tmux kill-session -t simulation_tmuxinator`
> - List sessions with `$ tmux list-sessions`
> - Kill all with `$ tmux kill-server`

> [!TIP]
> Docker shortcuts
> - `$ exit` or `Ctrl + d` will close the shell and stop the container (as it was started interactively with `-it`).
> - `Ctrl + p`  then  `Ctrl + q` detaches you from the container and leaves it running in the background
> - Re-attach with `$ docker attach <container_name_or_id>`

> [!TIP]
> Docker hygiene
```sh
docker ps -a # List containers
docker stop $(docker ps -q) # Stop all containers
docker container prune # Remove all stopped containers

docker images # List images
docker image prune # Remove untagged images
docker rmi <image_name_or_id> # Remove a specific image
```

---

## Part 3: Development with AAS

TBD

---

## Part 4: Deployment of AAS

TBD

---
> You've done a man's job, sir. I guess you're through, huh?

<!-- 

## WIP

### TODOs

- FIX MULTIDRONE ARDUPILOT SIMULATION

```sh
# Grant local dockers access to the X display server for GUI applications after every reboot
xhost +local:docker

docker run -it --rm \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged --net=host \
  simulation-image
# simulation-image starts $ tmuxinator start -p /git/resources/tmuxinator/simulation.yml.erb

docker run -it --rm \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged --net=host \
  aircraft-image
# aircraft-image starts $ tmuxinator start -p /git/resources/tmuxinator/aircraft.yml.erb

# # (optional) Revoke local dockers access to the X display server for GUI applications
# xhost -local:docker
```

### Networking

- NOTE: tmuxinator start -p /git/resources/simulation_tmuxinator.yml might have AP/GZ/QGC issue when wifi is on on the host, revise --net=host

- ArduPilot SITL architecture: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture
- ArduPilot UARTs: https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html
```sh
# on 192.168.1.30, add multiple outs for each SITL to use QGC (in the same container) and C++/ROS2 wrapped MAVSDK
./Tools/autotest/sim_vehicle.py -v ArduCopter --map --console --out=udp:192.168.1.30:14550 --out=udp:192.168.1.20:14540
# on 192.168.1.20, connect with
/git/MAVSDK/build/src/mavsdk_server/src/mavsdk_server udpin://0.0.0.0:14540
```
- PX$ SITL architecture: https://docs.px4.io/main/en/simulation/#sitl-simulation-environment
- PX4 XRCE-DDS architecture: https://docs.px4.io/main/en/middleware/uxrce_dds.html#architecture
```sh
# 42.42.42.xx, configure PX4 sitl with env variables
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_SYS_AUTOSTART=4001 PX4_UXRCE_DDS_NS="Drone1" PX4_UXRCE_DDS_AG_IP=42.42.42.20 PX4_UXRCE_DDS_PORT=8888 /git/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1
# on 42.42.42.20, connect with
MicroXRCEAgent udp4 -p 8888
```

ArduPilot Docker networking
```sh
docker network create drone-net

docker run -d --rm --network drone-net --name sitl-container \
  your-ardupilot-image \
  ./Tools/autotest/sim_vehicle.py -v ArduCopter --console \
  --out=udp:host.docker.internal:14550 \
  --out=udp:mavsdk-container:14540

docker run -d --rm --network drone-net --name mavsdk-container \
  your-mavsdk-image \
  /path/to/mavsdk_server udpin://0.0.0.0:14540
```

PX4 Docker networking
```sh
docker network create drone-net

docker run -d --rm --network drone-net --name xrce-agent \
  your-xrce-agent-image \
  MicroXRCEAgent udp4 -p 8888

docker run -d --rm --network drone-net --name px4-sitl \
  -e PX4_UXRCE_DDS_AG_IP=xrce-agent \
  -e PX4_SYS_AUTOSTART=4008 \
  -e PX4_UXRCE_DDS_NS="Drone1" \
  your-px4-image \
  /path/to/px4 -i 1
```

Inter drone serial communication (for Docker simulation and deployment)

```sh
# Create the IP network
docker network create my-app-net

# Create the virtual serial port pair using socat
socat -d -d pty,raw,echo=0,link=/tmp/port-a pty,raw,echo=0,link=/tmp/port-b &

docker run -d --rm \
  --network my-app-net \
  --name container-a \
  --device=/tmp/port-a:/dev/ttyS0 \
  your-app-image-a

docker run -d --rm \
  --network my-app-net \
  --name container-b \
  --device=/tmp/port-b:/dev/ttyS0 \
  your-app-image-b
```

Image processing from simulation to containers

```sh
# In the drone .sdf
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/demo</namespace>
    <remapping>image_raw:=color/image_raw</remapping>
  </ros>
  </plugin>

# In the simulation container
gst-launch-1.0 ros2imagesrc topic-name="/demo/color/image_raw" ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! \
    rtph264pay ! \
    udpsink host=yolo-container port=5000
```

```py
# In the YOLO container
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
# ... other imports for numpy, onnx, etc.

# GStreamer pipeline to receive, decode, and send to the application
pipeline_str = "udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink name=yolo_sink emit-signals=true"

# ... Code to launch the pipeline and a callback function for the 'new-sample' signal from appsink
# Inside the callback, you get the frame buffer and pass it to your YOLOv8 ONNX model.
```

### Simulation Resources

- https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html#x500-quadrotor-with-depth-camera-front-facing
- https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html#x500-quadrotor-with-2d-lidar 
- https://ardupilot.org/dev/docs/sitl-with-gazebo.html Iris quadcopter and a Zephyr delta-wing.
- https://github.com/nathanbowness/UAV-Object-Tracking


### Geospatial and Photogrammetry Resources
- https://support.pix4d.com/hc/en-us/articles/360000235126#OPF2
- https://github.com/softwareunderground/awesome-open-geoscience?tab=readme-ov-file
- https://github.com/sacridini/Awesome-Geospatial
- https://github.com/awesome-photogrammetry/awesome-photogrammetry?tab=readme-ov-file
- https://app.gazebosim.org/fuel/worlds
- https://aszabo.substack.com/p/zero-to-hero-creating-gazebo-worlds?utm_campaign=post&utm_medium=web
- https://github.com/AndrejOrsula/space_robotics_gz_envs
- https://github.com/domlysz/BlenderGIS
- https://cesium.com/platform/cesiumjs/ 

-->
