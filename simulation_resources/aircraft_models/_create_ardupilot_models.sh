#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <number_of_drones> <full_path_to_base_model_directory>"
  echo "Example: ./_create_ardupilot_models.sh 3 /git/simulation_resources/aircraft_models/iris_with_ardupilot"
  exit 1
fi

NUM_DRONES=$1
BASE_MODEL_PATH=$2
BASE_PORT=9002

# Get the model/directory name from the full path
AIRCRAFT_MODELS_PATH=$(dirname "$BASE_MODEL_PATH")
BASE_MODEL_NAME=$(basename "$BASE_MODEL_PATH")

if [ ! -d "$BASE_MODEL_PATH" ]; then
    echo "Error: Source directory '${BASE_MODEL_PATH}' not found."
    exit 1
fi

echo "Creating ${NUM_DRONES} model(s) from source '${BASE_MODEL_NAME}'..."

for i in $(seq 1 $NUM_DRONES); do

    NEW_MODEL_NAME="${BASE_MODEL_NAME}_${i}"
    NEW_MODEL_DIR="${AIRCRAFT_MODELS_PATH}/${NEW_MODEL_NAME}"

    mkdir "$NEW_MODEL_DIR"
    cp "$BASE_MODEL_PATH"/*.sdf "$BASE_MODEL_PATH"/*.config "$NEW_MODEL_DIR"/

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

    # Update the model name and fdm_port_in in the copied SDF file
    sed -i "s/<model name=\"${BASE_MODEL_NAME}\">/<model name=\"${NEW_MODEL_NAME}\">/g" "${AIRCRAFT_MODELS_PATH}/${NEW_MODEL_NAME}/model.sdf"
    sed -i "s/<fdm_port_in>${BASE_PORT}<\/fdm_port_in>/<fdm_port_in>$(($BASE_PORT + (i-1) * 10))<\/fdm_port_in>/g" "${AIRCRAFT_MODELS_PATH}/${NEW_MODEL_NAME}/model.sdf"

    # Copy and update the params file
    DEST_PARAMS="${NEW_MODEL_DIR}/ardupilot-4.6.params"
    cp "${BASE_MODEL_PATH}/ardupilot-4.6.params" "$DEST_PARAMS"

    if grep -q "SYSID_THISMAV" "$DEST_PARAMS"; then
        # If SYSID_THISMAV exists, replace its value
        sed -i "s/SYSID_THISMAV.*/SYSID_THISMAV $i/g" "$DEST_PARAMS"
    else
        # If SYSID_THISMAV doesn't exist, add it to the end of the file
        echo "SYSID_THISMAV $i" >> "$DEST_PARAMS"
    fi

done
