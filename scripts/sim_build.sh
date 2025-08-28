#!/bin/bash

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Create a folder (ignored by git) to clone GitHub repos
CLONE_DIR="${SCRIPT_DIR}/../github_clones"
mkdir -p "$CLONE_DIR"

REPOS=( # Format: "URL;BRANCH;LOCAL_DIR_NAME"
    # Simulation image
    "https://github.com/PX4/PX4-Autopilot.git;v1.16.0;PX4-Autopilot"
    "https://github.com/ArduPilot/ardupilot.git;Copter-4.6.2;ardupilot"
    "https://github.com/ArduPilot/ardupilot_gazebo.git;main;ardupilot_gazebo"
    # Aircraft image
    "https://github.com/PX4/px4_msgs.git;release/1.16;px4_msgs"
    "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git;master;Micro-XRCE-DDS-Agent"
    # "https://github.com/microsoft/onnxruntime.git;v1.22.1;onnxruntime" # Only for the deployment build
    "https://github.com/PRBonn/kiss-icp.git;main;kiss-icp"
)

for repo_info in "${REPOS[@]}"; do
    IFS=';' read -r url branch dir <<< "$repo_info" # Split the string into URL, BRANCH, and DIR
    TARGET_DIR="${CLONE_DIR}/${dir}"
    if [ -d "$TARGET_DIR" ]; then
        cd "$TARGET_DIR"
        BRANCH=$(git branch --show-current)
        TAGS=$(git tag --points-at HEAD)
        echo "There is a clone of ${dir} on branch: ${BRANCH}, tags: [${TAGS}]"
        # The script does not automatically pull changes for already cloned repos (as they should be on fixed tags)
        # git pull
        # git submodule update --init --recursive --depth 1
        cd "$CLONE_DIR"
    else
        echo "Clone not found, cloning ${dir}..."
        TEMP_DIR="${TARGET_DIR}_temp"     
        rm -rf "$TEMP_DIR" # Clean up any failed clone from a previous run   
        git clone --depth 1 --branch "$branch" --recursive "$url" "$TEMP_DIR" && mv "$TEMP_DIR" "$TARGET_DIR"
    fi
done

# The first build takes ~15' and creates a 21GB image (8GB for ros-humble-desktop with nvidia runtime, 10GB for PX4 and ArduPilot SITL)
docker build -t simulation-image -f "${SCRIPT_DIR}/docker/Dockerfile.simulation" "${SCRIPT_DIR}/.."

# The first build takes ~10' and creates a 16GB image (8GB for ros-humble-desktop with nvidia runtime, 7GB for YOLOv8, ONNX)
docker build -t aircraft-image -f "${SCRIPT_DIR}/docker/Dockerfile.aircraft" "${SCRIPT_DIR}/.."
