#!/usr/bin/env bash

# N.B.: do not use extend at the first source 

echo "*** Using mr3d_wss ***"

THIS_DIR="$(cd "$(dirname "$BASH_SOURCE")"; pwd)"
THIS_DIR=$(readlink -f $THIS_DIR)  # this reads the actual path if a symbolic directory is used
export MR3D_HOME=$THIS_DIR
echo MR3D_HOME: $MR3D_HOME

source ~/.bashrc 

# load the config environment variables
source $MR3D_HOME/config.sh --extend

for ws in "${LIST_OF_WSS[@]}"   # in config.sh 
do
    if [ -d $ws ]; then
        cd $ws 
            if [ -f devel/setup.bash ]; then
                source devel/setup.bash --extend
            fi
        cd ..
    fi
done

# https://answers.ros.org/question/266313/robot-model-not-showing-in-rviz/
# added for making the robots appear in RVIZ 
export LC_NUMERIC="en_US.UTF-8"

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"$MR3D_HOME/gazebo_models/PX4-SITL_gazebo-classic/models"

