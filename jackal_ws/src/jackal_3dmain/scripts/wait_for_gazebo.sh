#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "$BASH_SOURCE")"; pwd)"
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used

$SCRIPT_DIR/wait_for_rosmaster.sh 

CHECK=""
while true; do
    echo waiting for gazebo
    CHECK=$(rosnode list | grep gazebo)
    if [ -n "$CHECK" ]; then # esci dal ciclo 
    break 
    fi
    sleep 1
done 
echo gazebo started