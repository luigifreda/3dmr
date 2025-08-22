#!/usr/bin/env bash

STARTING_DIR=`pwd`
echo STARTING_DIR: $STARTING_DIR

set -e 

source ~/.bashrc

source source_all.bash 
cd $MR3D_HOME

echo "VREP_ROOT_DIR: $VREP_ROOT_DIR"

for ws in "${LIST_OF_WSS[@]}"   # in config.sh 
do
    if [ -d $ws ]; then
        cd $ws 
		if [ ! -f src/CMakeLists.txt ]; then
			cd src
			catkin_init_workspace
			cd ..	
		fi
		catkin build -DCMAKE_BUILD_TYPE=Release
		cd $MR3D_HOME
		source source_all.bash 
    fi
done

# check if the vrep plugin lib is synched
echo ""
echo "checking if we need to update VREP plugin..."
LIB_VREP=libv_repExtRos.so
if [ -f nav_ws/devel/lib/$LIB_VREP ]; then
	if [ ! -f $VREP_ROOT_DIR/$LIB_VREP ]; then
		echo "copying libv_repExtRos.so in $VREP_ROOT_DIR"
		sudo cp nav_ws/devel/lib/$LIB_VREP $VREP_ROOT_DIR		
	fi
	DIFF_LIB=$(diff nav_ws/devel/lib/$LIB_VREP $VREP_ROOT_DIR/$LIB_VREP)
	#echo "DIFF_LIB: $DIFF_LIB"
	if [ "$DIFF_LIB" != "" ]; then
		echo "copying libv_repExtRos.so in $VREP_ROOT_DIR"
		sudo cp nav_ws/devel/lib/$LIB_VREP $VREP_ROOT_DIR	
	else
		echo "$LIB_VREP synched"	
	fi
else
	echo "you still need to compile vrep package"
fi
echo "done"

# go back to starting dir 
cd $STARTING_DIR


