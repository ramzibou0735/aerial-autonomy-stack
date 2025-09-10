#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <num_quads> <num_vtols>"
  echo "Example: ./_create_ardupilot_models.sh 2 1"
  exit 1
fi

NUM_QUADS=$1
NUM_VTOLS=$2

BASE_PORT=9002

# Paths to the base models
QUAD_MODEL_PATH="/simulation_resources/aircraft_models/iris_with_ardupilot"
VTOL_MODEL_PATH="/simulation_resources/aircraft_models/alti_transition_quad"

# Check if model directories exist
if [ ! -d "$QUAD_MODEL_PATH" ] && [ "$NUM_QUADS" -gt 0 ]; then
    echo "Error: Quad model directory '${QUAD_MODEL_PATH}' not found."
    exit 1
fi

if [ ! -d "$VTOL_MODEL_PATH" ] && [ "$NUM_VTOLS" -gt 0 ]; then
    echo "Error: VTOL model directory '${VTOL_MODEL_PATH}' not found."
    exit 1
fi

echo "Creating ${NUM_QUADS} quadcopter(s) and ${NUM_VTOLS} VTOL(s)..."

# Counter for unique port and model IDs
DRONE_COUNT=0

# Loop for quads
for i in $(seq 1 $NUM_QUADS); do
    DRONE_COUNT=$((DRONE_COUNT + 1))
    
    BASE_MODEL_NAME=$(basename "$QUAD_MODEL_PATH")
    NEW_MODEL_NAME="${BASE_MODEL_NAME}_${DRONE_COUNT}"
    NEW_MODEL_DIR="${QUAD_MODEL_PATH}/../${NEW_MODEL_NAME}"

    mkdir -p "$NEW_MODEL_DIR"
    cp "$QUAD_MODEL_PATH"/*.sdf "$QUAD_MODEL_PATH"/*.config "$NEW_MODEL_DIR"/
    
    # Create .config file to let gz sim include the model
    CONFIG_FILE="${NEW_MODEL_DIR}/model.config"
    echo "<?xml version='1.0'?>" > "$CONFIG_FILE"
    echo "<model>" >> "$CONFIG_FILE"
    echo "  <name>${NEW_MODEL_NAME}</name>" >> "$CONFIG_FILE"
    echo "  <version>1.0</version>" >> "$CONFIG_FILE"
    echo "  <sdf version='1.9'>model.sdf</sdf>" >> "$CONFIG_FILE"
    echo "  <description>" >> "$CONFIG_FILE"
    echo "    A dynamically generated model of ${NEW_MODEL_NAME}." >> "$CONFIG_FILE"
    echo "  </description>" >> "$CONFIG_FILE"
    echo "</model>" >> "$CONFIG_FILE"

    sed -i "s/<model name=\"${BASE_MODEL_NAME}\">/<model name=\"${NEW_MODEL_NAME}\">/g" "${NEW_MODEL_DIR}/model.sdf"
    sed -i "s/<fdm_port_in>${BASE_PORT}<\/fdm_port_in>/<fdm_port_in>$(($BASE_PORT + ($DRONE_COUNT - 1) * 10))<\/fdm_port_in>/g" "${NEW_MODEL_DIR}/model.sdf"

    DEST_PARAMS="${NEW_MODEL_DIR}/ardupilot-4.6.params"
    cp "${QUAD_MODEL_PATH}/ardupilot-4.6.params" "$DEST_PARAMS"

done

# Loop for VTOLs
for i in $(seq 1 $NUM_VTOLS); do
    DRONE_COUNT=$((DRONE_COUNT + 1))
    
    BASE_MODEL_NAME=$(basename "$VTOL_MODEL_PATH")
    NEW_MODEL_NAME="${BASE_MODEL_NAME}_${DRONE_COUNT}"
    NEW_MODEL_DIR="${VTOL_MODEL_PATH}/../${NEW_MODEL_NAME}"

    mkdir -p "$NEW_MODEL_DIR"
    cp "$VTOL_MODEL_PATH"/*.sdf "$VTOL_MODEL_PATH"/*.config "$NEW_MODEL_DIR"/

    CONFIG_FILE="${NEW_MODEL_DIR}/model.config"
    echo "<?xml version='1.0'?>" > "$CONFIG_FILE"
    echo "<model>" >> "$CONFIG_FILE"
    echo "  <name>${NEW_MODEL_NAME}</name>" >> "$CONFIG_FILE"
    echo "  <version>1.0</version>" >> "$CONFIG_FILE"
    echo "  <sdf version='1.9'>model.sdf</sdf>" >> "$CONFIG_FILE"
    echo "  <description>" >> "$CONFIG_FILE"
    echo "    A dynamically generated model of ${NEW_MODEL_NAME}." >> "$CONFIG_FILE"
    echo "  </description>" >> "$CONFIG_FILE"
    echo "</model>" >> "$CONFIG_FILE"

    sed -i "s/<model name=\"${BASE_MODEL_NAME}\">/<model name=\"${NEW_MODEL_NAME}\">/g" "${NEW_MODEL_DIR}/model.sdf"
    sed -i "s/<fdm_port_in>${BASE_PORT}<\/fdm_port_in>/<fdm_port_in>$(($BASE_PORT + ($DRONE_COUNT - 1) * 10))<\/fdm_port_in>/g" "${NEW_MODEL_DIR}/model.sdf"

    DEST_PARAMS="${NEW_MODEL_DIR}/ardupilot-4.6.params"
    cp "${VTOL_MODEL_PATH}/ardupilot-4.6.params" "$DEST_PARAMS"

done

echo "Done."