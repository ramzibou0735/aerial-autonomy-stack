#!/bin/bash

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# The first build takes ~15' and creates a 19GB image (8GB for ros-humble-desktop with nvidia runtime, 9GB for PX4 and ArduPilot SITL)
docker build -t simulation-image -f "${SCRIPT_DIR}/../docker/Dockerfile.simulation" "${SCRIPT_DIR}/.."

# The first build takes ~15' and creates a 16GB image (8GB for ros-humble-desktop with nvidia runtime, 7GB for YOLOv8, ONNX)
docker build -t aircraft-image -f "${SCRIPT_DIR}/../docker/Dockerfile.aircraft" "${SCRIPT_DIR}/.."
