#!/usr/bin/env bash

CURRENT_DIR=`pwd`
CURRENT_DIR=$(readlink -f $CURRENT_DIR)  # this reads the actual path if a symbolic directory is used
echo "current dir: $CURRENT_DIR"
cd $CURRENT_DIR # this brings us in the actual used folder (not the symbolic one)

cd ..
source source_all.bash 
cd $CURRENT_DIR
#source devel/setup.bash

catkin build -DCMAKE_BUILD_TYPE=Release "$@"
