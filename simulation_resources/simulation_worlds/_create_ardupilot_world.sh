#!/bin/bash

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <number_of_drones> <base_model_name> <full_path_to_empty_world>"
  echo "Example: ./_create_ardupilot_world.sh 3 iris_with_ardupilot /git/simulation_resources/simulation_worlds/empty_ardupilot.sdf"
  exit 1
fi

NUM_DRONES=$1
BASE_MODEL_NAME=$2
BASE_WORLD_WITH_PATH=$3

MODEL_POSE_STRING="1 0 0 0"
case "$BASE_MODEL_NAME" in
  iris_with_ardupilot)
    MODEL_POSE_STRING="0.195 0 0 0"
    ;;
  zephyr_with_ardupilot)
    MODEL_POSE_STRING="0.422 -90 0 180"
    ;;
  *)
    # Use the default for any other model
    ;;
esac

# Create a copy of the template to work on
BASE_WORLD_DIR=$(dirname "$BASE_WORLD_WITH_PATH")
OUTPUT_FILE="${BASE_WORLD_DIR}/populated_ardupilot.sdf"
cp "$BASE_WORLD_WITH_PATH" "$OUTPUT_FILE"

# IMPORTANT: this replaces the whole <physics> block with Ardupilot's
ARDUPILOT_PHYSICS="    <physics name=\"1ms\" type=\"ignore\">\n      <max_step_size>0.001<\/max_step_size>\n      <real_time_factor>1.0<\/real_time_factor>\n    <\/physics>"
sed -i -e "/<physics/,/<\/physics>/c\\
${ARDUPILOT_PHYSICS}" "$OUTPUT_FILE"

# This loop builds a single string containing all the <include> blocks
ALL_MODELS_XML=""
for i in $(seq 1 $NUM_DRONES); do
  MODEL_XML="    <include>\n      <uri>model://${BASE_MODEL_NAME}_${i}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( (i-1) * 2 )) ${MODEL_POSE_STRING}</pose>\n    </include>\n"
  ALL_MODELS_XML+=$MODEL_XML
done

# Read the file, replace the tag, and write the content back out
WORLD_CONTENT=$(cat "$OUTPUT_FILE")
echo "${WORLD_CONTENT//'</world>'/"$ALL_MODELS_XML</world>"}" > "$OUTPUT_FILE"
