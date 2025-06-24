#!/bin/bash

NUM_INSTANCES=2
for i in $(seq 0 $((NUM_INSTANCES - 1))); do
  PX4_INSTANCE_DIR=/px4/build/instance${i}
  PX4_SIM_PORT=$((14540 + i))
  RTPS_PORT=$((8888 + i))
  AGENT_HOST="drone_sitl_$i"

  echo "Launching PX4 instance $i -> $AGENT_HOST:$RTPS_PORT"

  PX4_SYS_AUTOSTART=10016 \
  PX4_SIM_HOST_ADDR=127.0.0.1 \
  PX4_SIM_PORT=$PX4_SIM_PORT \
  micrortps_client_udp_port=$RTPS_PORT \
    make -C /px4 px4_sitl_rtps none_iris \
      BUILD_DIR=$PX4_INSTANCE_DIR &

done

wait