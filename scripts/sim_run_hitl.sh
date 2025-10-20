#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set up the simulation
NUM_QUADS="${NUM_QUADS:-1}" # Number of quadcopters (default = 1)
NUM_VTOLS="${NUM_VTOLS:-0}" # Number of VTOLs (default = 0)
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
WORLD="${WORLD:-impalpable_greyness}" # Options: impalpable_greyness (default), apple_orchard, shibuya_crossing, swiss_town
HEADLESS="${HEADLESS:-false}" # Options: true, false (default)
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false
SUBNET_PREFIX="${SUBNET_PREFIX:-42.42}" # Subnet prefix, e.g., 42.42 (default), 192.168, etc.

# Detect the environment (Ubuntu/GNOME, WSL, etc.)
if command -v gnome-terminal >/dev/null 2>&1 && [ -n "$XDG_CURRENT_DESKTOP" ]; then
  DESK_ENV="gnome"
else
  echo "Unsupported environment" 
  exit 1
fi
echo "Desktop environment: $DESK_ENV"

# Grant access to the X server
if command -v xhost >/dev/null 2>&1; then 
  xhost +local:docker
fi

# Get primary display dimensions
get_primary_display_info() {
  local resolution=$(xrandr 2>/dev/null | grep " connected primary" | grep -oE '[0-9]+x[0-9]+' | head -1)
  if [[ ! "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
    resolution=$(xrandr 2>/dev/null | grep " connected" | grep -oE '[0-9]+x[0-9]+' | head -1) # Fallback
  fi
  if [[ "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
    SCREEN_WIDTH=$(echo "$resolution" | cut -d'x' -f1)
    SCREEN_HEIGHT=$(echo "$resolution" | cut -d'x' -f2)
    echo "Detected display: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
  else
    SCREEN_WIDTH=1920
    SCREEN_HEIGHT=1080
    echo "Fallback resolution to ${SCREEN_WIDTH}x${SCREEN_HEIGHT} default"
  fi
}
get_primary_display_info

# Setup terminal dimensions
TERM_COLS=80
TERM_ROWS=32
FONT_SIZE=10
calculate_terminal_position() {
  local drone_id=$1
  SCREEN_SCALE=$((SCREEN_HEIGHT * 100 / 1080)) # Full HD = 100%
  X_POS=$(( (50 + drone_id * 50) * SCREEN_SCALE / 100 ))
  Y_POS=$(( (drone_id * 125) * SCREEN_SCALE / 100 ))
}

# Enable Shift+Ctrl+c, Shift+Ctrl+v copy-paste in xterm
XTERM_CONFIG_ARGS=(
  -xrm 'XTerm*selectToClipboard: true'
  -xrm 'XTerm*VT100.Translations: #override \
    Ctrl Shift <Key>C: copy-selection(CLIPBOARD) \n\
    Ctrl Shift <Key>V: insert-selection(CLIPBOARD)'
)

# Launch the simulation container
DOCKER_CMD="docker run -it --rm \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
  --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS \
  --env WORLD=$WORLD --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env SIMULATED_TIME=true \
  --env SUBNET_PREFIX=$SUBNET_PREFIX \
  --net=host \
  --privileged \
  --name simulation-container \
  simulation-image"
calculate_terminal_position 0
xterm "${XTERM_CONFIG_ARGS[@]}" -title "Simulation" -fa Monospace -fs $FONT_SIZE -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &
