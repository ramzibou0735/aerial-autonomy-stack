#!/bin/bash

if [ "$#" -ne 5 ]; then
  echo "Usage: $0 <num_rows> <row_separation_m> <row_length_m> <num_trees_per_row> <slope_drop_m>"
  echo "Example: ./procedural_trees.sh 14 6 400 18 6"
  exit 1
fi

NUM_ROWS=$1
ROW_SEPARATION=$2
ROW_LENGTH=$3
NUM_TREES_PER_ROW=$4
SLOPE_DROP=$5
OUTPUT_FILE="model.sdf"

# Slope Calculation
SLOPE_PER_METER=$(echo "scale=4; $SLOPE_DROP / $ROW_LENGTH" | bc)

# Header
cat <<EOF > "$OUTPUT_FILE"
<?xml version="1.0"?>
<sdf version="1.10">
  <model name="orchard">
    <static>true</static>
EOF

# Main Loop
echo "Generating a grid of ${NUM_ROWS} rows with ${NUM_TREES_PER_ROW} trees per row..."

for c in $(seq 0 $(($NUM_TREES_PER_ROW - 1))); do
  for r in $(seq 0 $(($NUM_ROWS - 1))); do
    # Non-Linear Spacing
    # Normalize the tree's position from 0 to 1
    NORM_POS=$(echo "scale=4; $c / ($NUM_TREES_PER_ROW - 1)" | bc)
    # Apply a quadratic function (x^2) to create progressive spacing
    SPACING_FACTOR=$(echo "scale=4; 1 - sqrt(1 - $NORM_POS)" | bc -l)
    # Calculate the final X position
    POS_X=$(echo "scale=4; $SPACING_FACTOR * $ROW_LENGTH" | bc)
    
    # Calculate Y and Z positions
    POS_Y=$(echo "$r * $ROW_SEPARATION" | bc)
    POS_Z=$(echo "-1 * $POS_X * $SLOPE_PER_METER" | bc)

    # Append the link block for each tree
    cat <<EOF >> "$OUTPUT_FILE"
    <link name='tree_r${r}_c${c}'>
      <pose>${POS_X} ${POS_Y} ${POS_Z} 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>100</mass>
        <inertia>
          <ixx>1</ixx>
          <iyy>1</iyy>
          <izz>1</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <pose degrees="true">-0.25 0 1.5 0 0 0</pose>
        <geometry>
            <box>
              <size>0.5 0.5 3.0</size>
            </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <pose degrees="true">0 0 0 90 0 0</pose>
        <geometry>
          <mesh>
            <scale>.025 .025 .025</scale>
            <uri>model://orchard_trees_near/meshes/tree.fbx</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
EOF
  done
done

# SDF Footer
cat <<EOF >> "$OUTPUT_FILE"
  </model>
</sdf>
EOF

echo "Done. World created at ${OUTPUT_FILE}"
