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
MODE="${MODE:-}" # Options: empty (default), dev, ...

# Detect the environment (Ubuntu/GNOME, WSL, etc.)
if command -v gnome-terminal >/dev/null 2>&1 && [ -n "$XDG_CURRENT_DESKTOP" ]; then
  DESK_ENV="gnome"
elif grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
  DESK_ENV="wsl"
else
  echo "Unsupported environment" 
  exit 1
fi
echo "Desktop environment: $DESK_ENV"

# Cleanup function
cleanup() {
  DOCKER_PIDS=$(pgrep -f "docker run.*simulation-image|docker run.*aircraft-image" 2>/dev/null || true)
  SIMULATION_CONTAINERS=$(docker ps -a -q --filter name=simulation-container 2>/dev/null || true)
  AIRCRAFT_CONTAINERS=$(docker ps -a -q --filter name=aircraft-container 2>/dev/null || true)
  echo "Stopping Docker containers (this will take a few seconds)..."
  if [ -n "$SIMULATION_CONTAINERS" ]; then
    docker stop $SIMULATION_CONTAINERS
  fi
  if [ -n "$AIRCRAFT_CONTAINERS" ]; then
    docker stop $AIRCRAFT_CONTAINERS
  fi
  docker network rm aas-network 2>/dev/null && echo "Removed aas-network" || echo "Network aas-network not found or already removed"
  if [ -n "$DOCKER_PIDS" ]; then
    for dpid in $DOCKER_PIDS; do
      PARENT_PID=$(ps -o ppid= -p $dpid 2>/dev/null | tr -d ' ') # Determine process pids with a parent pid
      if [ -n "$PARENT_PID" ]; then
        echo "Killing terminal process $dpid"
        kill $dpid
      fi
    done
  fi
  echo "All-clear"
}
# Set trap to cleanup on script interruption (Ctrl+C, etc.)
trap cleanup EXIT INT TERM

# Initialize an empty variable for the flags
MODE_SIM_OPTS=""
MODE_AIR_OPTS=""
case "$MODE" in
  dev)
    # In dev mode, resources and workspaces are mounted from the host
    SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    MODE_SIM_OPTS="--entrypoint /bin/bash"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_resources/:/simulation_resources:cached"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src:/ros2_ws/src:cached"
    MODE_AIR_OPTS="--entrypoint /bin/bash"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_resources/:/aircraft_resources:cached"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_ws/src:/ros2_ws/src:cached"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src/ground_system_msgs:/ros2_ws/src/ground_system_msgs:cached"
    ;;
  *)
    MODE_SIM_OPTS=""
    MODE_AIR_OPTS=""
    ;;
esac

# Grant access to the X server
if command -v xhost >/dev/null 2>&1; then 
  xhost +local:docker
fi

# Create network
NETWORK_NAME="aas-network"
docker network inspect "$NETWORK_NAME" >/dev/null 2>&1 || docker network create --subnet=42.42.0.0/16 "$NETWORK_NAME"

# WSL-specific options
WSL_OPTS="--env WAYLAND_DISPLAY=$WAYLAND_DISPLAY --env PULSE_SERVER=$PULSE_SERVER --volume /usr/lib/wsl:/usr/lib/wsl \
--env LD_LIBRARY_PATH=/usr/lib/wsl/lib --env LIBGL_ALWAYS_SOFTWARE=0 --env __GLX_VENDOR_LIBRARY_NAME=nvidia"

# Get primary display dimensions (for multi-monitor)
get_primary_display_info() {
    if command -v xrandr >/dev/null 2>&1; then
        # Get primary display info
        local primary_info=$(xrandr 2>/dev/null | grep " connected primary" | head -1)
        if [[ -n "$primary_info" ]]; then
            # Extract resolution from primary display
            local resolution=$(echo "$primary_info" | grep -oE '[0-9]+x[0-9]+' | head -1)
            if [[ -n "$resolution" && "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
                SCREEN_WIDTH=$(echo "$resolution" | cut -d'x' -f1)
                SCREEN_HEIGHT=$(echo "$resolution" | cut -d'x' -f2)
                echo "Using primary display: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
                return 0
            fi
        fi
        # Fallback: get first active display
        local resolution=$(xrandr 2>/dev/null | grep -E '^\s*[0-9]+x[0-9]+.*\*' | head -n1 | awk '{print $1}')
        if [[ -n "$resolution" && "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
            SCREEN_WIDTH=$(echo "$resolution" | cut -d'x' -f1)
            SCREEN_HEIGHT=$(echo "$resolution" | cut -d'x' -f2)
            echo "Using first active display: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
            return 0
        fi
    fi
    # Fallback to xdpyinfo (gets combined resolution on multi-monitor)
    if command -v xdpyinfo >/dev/null 2>&1; then
        local resolution=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}')
        if [[ -n "$resolution" && "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
            SCREEN_WIDTH=$(echo "$resolution" | cut -d'x' -f1)
            SCREEN_HEIGHT=$(echo "$resolution" | cut -d'x' -f2)
            echo "Using combined display resolution: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
            return 0
        fi
    fi
    # Default fallback
    SCREEN_WIDTH=1920
    SCREEN_HEIGHT=1080
    echo "Using default resolution: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
}
# Dynamic character size estimation based on screen resolution
get_char_dimensions() {
    local dpi_scale=1.0
    # Estimate DPI scaling based on resolution
    if [ "$SCREEN_WIDTH" -ge 2560 ] || [ "$SCREEN_HEIGHT" -ge 1440 ]; then
        # High resolution display (1440p, 4K, etc.)
        dpi_scale=1.5
    elif [ "$SCREEN_WIDTH" -ge 3840 ] || [ "$SCREEN_HEIGHT" -ge 2160 ]; then
        # 4K and above
        dpi_scale=2.0
    fi
    # Base character dimensions for Monospace -fs 10
    local base_char_width=6
    local base_char_height=13
    # Apply scaling
    CHAR_WIDTH=$(echo "$base_char_width * $dpi_scale" | bc -l | cut -d. -f1)
    CHAR_HEIGHT=$(echo "$base_char_height * $dpi_scale" | bc -l | cut -d. -f1)
    # Fallback if bc is not available
    if [ -z "$CHAR_WIDTH" ] || [ "$CHAR_WIDTH" -eq 0 ]; then
        CHAR_WIDTH=$(awk "BEGIN {printf \"%.0f\", $base_char_width * $dpi_scale}")
    fi
    if [ -z "$CHAR_HEIGHT" ] || [ "$CHAR_HEIGHT" -eq 0 ]; then
        CHAR_HEIGHT=$(awk "BEGIN {printf \"%.0f\", $base_char_height * $dpi_scale}")
    fi
    echo "Character dimensions: ${CHAR_WIDTH}x${CHAR_HEIGHT} (scale: $dpi_scale)"
}
# Adaptive terminal sizing
get_terminal_dimensions() {
    # Adjust terminal size based on screen resolution
    if [ "$SCREEN_WIDTH" -ge 2560 ]; then
        # Larger terminals for high-res displays
        TERM_COLS=100
        TERM_ROWS=35
    elif [ "$SCREEN_WIDTH" -ge 1920 ]; then
        # Standard size for 1080p+
        TERM_COLS=80
        TERM_ROWS=30
    else
        # Smaller terminals for lower resolution
        TERM_COLS=70
        TERM_ROWS=25
    fi
    TERM_WIDTH=$((TERM_COLS * CHAR_WIDTH))
    TERM_HEIGHT=$((TERM_ROWS * CHAR_HEIGHT))
    echo "Terminal size: ${TERM_COLS}x${TERM_ROWS} (${TERM_WIDTH}x${TERM_HEIGHT} pixels)"
}
# Improved positioning with margin consideration
calculate_terminal_position() {
    local drone_id=$1 # 0-based drone ID (0 = simulation, 1+ = drones)
    local total_terminals=$2
    # Add margins for better spacing
    local margin=20
    local usable_height=$((SCREEN_HEIGHT - margin * 2))
    local usable_width=$((SCREEN_WIDTH - margin * 2))
    # X position logic (odd/even based on drone_id)
    if [ $((drone_id % 2)) -eq 1 ]; then
        # Odd: offset from right edge
        X_POS=$((SCREEN_WIDTH - TERM_WIDTH - 200 - margin))
    else
        # Even: at right edge
        X_POS=$((SCREEN_WIDTH - TERM_WIDTH - margin))
    fi
    # Y position: distributed vertically with margins
    if [ "$total_terminals" -eq 1 ]; then
        Y_POS=$(( (SCREEN_HEIGHT - TERM_HEIGHT) / 2 ))
    else
        # Use drone_id directly (0, 1, 2, ...) for even spacing
        Y_POS=$(( margin + drone_id * usable_height / (total_terminals - 1) ))
        # Ensure we don't go off screen
        if [ $((Y_POS + TERM_HEIGHT)) -gt $((SCREEN_HEIGHT - margin)) ]; then
            Y_POS=$((SCREEN_HEIGHT - TERM_HEIGHT - margin))
        fi
    fi
}
get_primary_display_info
get_char_dimensions  
get_terminal_dimensions
NUM_TERMINALS=$((NUM_QUADS + NUM_VTOLS + 1))

# Initialize a counter for the drone IDs
DRONE_ID=0

# Launch the simulation container
DOCKER_CMD="echo 'Launching Simulation Container...'; \
  docker run -it --rm \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
  --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS \
  --env WORLD=$WORLD --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env SIMULATED_TIME=true \
  --net=aas-network --ip=42.42.1.99 \
  --privileged \
  --name simulation-container"
# Add WSL-specific options and complete the command
if [[ "$DESK_ENV" == "wsl" ]]; then
    DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
fi
DOCKER_CMD="$DOCKER_CMD ${MODE_SIM_OPTS} simulation-image"
calculate_terminal_position $DRONE_ID $NUM_TERMINALS
xterm -title "Simulation" -fa Monospace -fs 10 -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &

# Launch the quad containers
for i in $(seq 1 $NUM_QUADS); do
  DRONE_ID=$((DRONE_ID + 1))
  sleep 1.5 # Limit resource usage
  DOCKER_CMD="echo 'Launching Quadcopter Container $DRONE_ID...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
      --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
      --env DRONE_TYPE=quad \
      --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
      --env SIMULATED_TIME=true \
      --net=aas-network --ip=42.42.1.$DRONE_ID \
      --privileged \
      --name aircraft-container_$DRONE_ID"
  # Add WSL-specific options and complete the command
  if [[ "$DESK_ENV" == "wsl" ]]; then
      DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
  fi
  DOCKER_CMD="$DOCKER_CMD ${MODE_SIM_OPTS} aircraft-image"
  calculate_terminal_position $DRONE_ID $NUM_TERMINALS
  xterm -title "Quad $DRONE_ID" -fa Monospace -fs 10 -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &
done

# Launch the vtol containers
for i in $(seq 1 $NUM_VTOLS); do
  DRONE_ID=$((DRONE_ID + 1))
  sleep 1.5 # Limit resource usage
  DOCKER_CMD="echo 'Launching VTOL Container $DRONE_ID...'; \
    docker run -it --rm \
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
      --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
      --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
      --env DRONE_TYPE=vtol \
      --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
      --env SIMULATED_TIME=true \
      --net=aas-network --ip=42.42.1.$DRONE_ID \
      --privileged \
      --name aircraft-container_$DRONE_ID"
  # Add WSL-specific options and complete the command
  if [[ "$DESK_ENV" == "wsl" ]]; then
      DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
  fi
  DOCKER_CMD="$DOCKER_CMD ${MODE_SIM_OPTS} aircraft-image"
  calculate_terminal_position $DRONE_ID $NUM_TERMINALS
  xterm -title "VTOL $DRONE_ID" -fa Monospace -fs 10 -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &
done

echo "Fly, my pretties, fly!"
echo "Press any key to stop all containers and close the terminals..."
read -n 1 -s # Wait for user input
