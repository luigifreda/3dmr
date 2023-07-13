#!/usr/bin/env bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

BAG_FILE="/media/luigi/Samsung_T5/rgbd_datasets2/euroc/V1_01_easy.bag"


# ======================================================================
if [[ -z "${MR3D_HOME}" ]]; then
    echo "ERROR: missing env var MR3D_HOME"
    echo "please, source source_all.bash in the main 3DMR folder"
    exit 
else
    source $MR3D_HOME/source_all.bash
fi

# ======================================================================


rosparam set use_sim_time true


# ======================================================================

xterm -e "echo euroc launcher; roslaunch voxblox_ros euroc_dataset.launch play_bag:=false; bash" &
sleep 3


# # ======================================================================

xterm -e "echo play euroc launcher; roslaunch voxblox_ros playbag.launch bag_file:=$BAG_FILE; bash" &


# ======================================================================

 
xterm -e "echo RVIZ ; roslaunch voxblox_ros rviz.launch ; bash" &


# ======================================================================

echo "DONE "


