#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
exec micro-ros-agent udp4 --port $XRCE_PORT