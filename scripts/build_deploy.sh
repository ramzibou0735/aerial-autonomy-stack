#!/bin/bash

# Note that this should run on Orin to build for arm64

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# The first build takes ~1h (mostly to build onnxruntime-gpu from source) and creates an 18GB image
docker build -t aircraft-image -f "${SCRIPT_DIR}/../docker/Dockerfile.aircraft" "${SCRIPT_DIR}/.."
