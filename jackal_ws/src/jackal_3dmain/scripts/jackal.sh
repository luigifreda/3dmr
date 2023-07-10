#!/usr/bin/env bash

export world_name="jackal_baylands.world" # one of the worlds contained in the folder $(find jackal_3dmain)/worlds/ 
export gazebo_gui=true

# select only one of the lidars!
export OS0=true 
export OS1=false 
export VLP16=false

export imuRate=400

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"/media/microsd/ubuntu18/Work/robotics_wss/gazebo_worlds/PX4-SITL_gazebo-classic/models"

# if there is an input argument, override the file name 
if [ $# -eq 1 ]; then
    world_name=$1
fi

if [[ -z "${LAUNCH_GAZEBO_MODE_ENV}" ]]; then
  true #using default value
else
  gazebo_gui="${LAUNCH_GAZEBO_MODE_ENV}"
  echo "set from env var - LAUNCH_GAZEBO_MODE: $LAUNCH_GAZEBO_MODE"  
fi

xterm -T "roscore" -e "echo roscore; roscore; bash" &
sleep 1

rosparam set use_sim_time true

# launch the main world in gazebo 
xterm -T "gazebo" -e "echo gazebo; roslaunch jackal_3dmain gazebo.launch world_name:=$world_name gui:=$gazebo_gui; bash" &
sleep 13

# inject the robot model into the opened gazebo world 
xterm -T "jackal" -e "echo jackal; roslaunch jackal_3dmain jackal.launch OS0:=$OS0 OS1:=$OS1 VLP16:=$VLP16 imuRate:=$imuRate; bash" &
