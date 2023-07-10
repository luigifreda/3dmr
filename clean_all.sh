#! /bin/bash
set +e  # disable "exit immediately"

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
        echo cleaning $ws 
        rm -Rf build/ devel/ logs/
        if [ -d ".catkin_tools" ]; then 
            rm -Rf .catkin_tools
        fi 
        #rm -Rf build 
        cd ..
    fi
done

set -e # enable "exit immediately"

