#!/bin/bash

if [[ -n "$NUM_DRONES" && "$NUM_DRONES" =~ ^[0-9]+$ ]]; then
    
    if [ "$AUTOPILOT" == "ardupilot" ]; then
        for i in $(seq 1 $NUM_DRONES); do
            filename=$(printf "%08d.BIN" "$i") # Create filenames like 00000001.BIN, 00000002.BIN, etc.
            logfile="/logs/$filename"
            if [ -f "$logfile" ]; then
                echo "Opening log for drone $i: $logfile"
                MAVExplorer.py "$logfile"  &
            else
                echo "Log file not found for drone $i: $logfile"
            fi
        done

    elif [ "$AUTOPILOT" == "px4" ]; then
        for i in $(seq 0 $((NUM_DRONES - 1))); do
            log_dir="/git/PX4-Autopilot/build/px4_sitl_default/rootfs/$i/log"
            # Find the latest dated folder and then the latest .ulg file inside it
            latest_date_dir=$(ls -td "$log_dir"/* | head -n 1)
            if [ -d "$latest_date_dir" ]; then
                latest_ulg=$(ls -t "$latest_date_dir"/*.ulg | head -n 1)
                if [ -n "$latest_ulg" ]; then
                    echo "Latest .ulg log for PX4 drone $i is: $(basename "$latest_ulg")"
                    echo "Opening PX4 log for drone $i: $latest_ulg"
                    # TODO
                else
                    echo "No .ulg logs found for PX4 drone $i in: $latest_date_dir"
                fi
            else
                echo "No log directory found for PX4 drone $i at: $log_dir"
            fi
        done

    else
        echo "Unknown AUTOPILOT"
        exit 0
    fi

else
    echo "Error: NUM_DRONES environment variable is not set or is not a valid number."
    exit 1
fi