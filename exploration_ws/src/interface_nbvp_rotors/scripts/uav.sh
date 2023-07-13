#!/bin/bash

echo "usage: ./${0##*/} <world-name>"

export LAUNCH_GAZEBO=1      # 1 automatic launch of GAZEBO, 0 no automatic launch
export LAUNCH_GAZEBO_MODE=1 # 0 launch GAZEBO in headless mode 
	                        # 1 launch GAZEBO in normal mode 

# ================  Values set by default if none is given ============
# the following worlds are available in the folder "jackal_ws/src/gazebo/jackal/jackal_3dmain/worlds"
# jackal_baylands 
# office

# select the world for the exploration simulation: specify here the world name you want
WORLD_NAME=$(rospack find interface_nbvp_rotors)/worlds/flat.world
export ENABLE_MULTI=1 

#======================================================================

# check environment variables 

if [[ -z "${MR3D_HOME}" ]]; then
    echo "ERROR: missing env var MR3D_HOME"
    echo "please, source source_all.bash in the main 3DMR folder"
    exit 
else
    source $MR3D_HOME/source_all.bash
fi

########################################################################################################

# check if we want to set the world from input and override
export WORLD_NAME_FROM_INPUT=$1
if [[ -n "$WORLD_NAME_FROM_INPUT" ]]; then 
    echo "setting world from input"
    WORLD_NAME=$WORLD_NAME_FROM_INPUT    
fi
echo "WORLD_NAME: $WORLD_NAME"

if [[ -z "${ENABLE_MULTI_ENV}" ]]; then
  #using default value
  true
else
  ENABLE_MULTI="${ENABLE_MULTI_ENV}"
  echo "set from env var - ENABLE_MULTI: $ENABLE_MULTI_ENV"
fi

########################################################################################################
#- GAZEBO

if [ $LAUNCH_GAZEBO -eq 1 ]; then

    echo "launching gazebo"
    open_term GAZEBOMAIN "echo gazebo main; roslaunch interface_nbvp_rotors gazebo.launch world_name:=$WORLD_NAME gui:=$LAUNCH_GAZEBO_MODE; bash" &
    rosrun interface_nbvp_rotors wait_for_gazebo.sh
    sleep 10   
fi 

## VOLUMETRIC EXPLORATION 
if [ $ENABLE_MULTI -eq 1 ]; then
    open_term uav_exploration "echo flat multi-robot exploration; roslaunch interface_nbvp_rotors multiagent_flat_exploration.launch enable_gazebo:=false; bash" &
else 
    open_term uav_exploration "echo flat exploration; roslaunch interface_nbvp_rotors flat_exploration.launch enable_gazebo:=false; bash" &
fi 
