#!/usr/bin/env bash

# you can use this script to install gazebo, some models and its dependencies 

# ====================================================

set -x

function print_blue(){
	printf "\033[34;1m"
	printf "$@ \n"
	printf "\033[0m"
}

# ====================================================

print_blue '================================================'
print_blue 'Installing gazebo models'
print_blue '================================================'

set -e
set -x 

STARTING_DIR=`pwd`
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


sudo apt install gazebo11 libgazebo11 libgazebo11-dev -y 


# Setup the default gazebo folder storing your gazebo models:
cd ~
if [ ! -d ".gazebo" ]; then 
    mkdir .gazebo
fi 
cd $SCRIPT_DIR

if [ ! -d "gazebo_models" ]; then
	mkdir gazebo_models
fi
cd gazebo_models 

if [ ! -d gazebo_models ]; then
    echo cloning gazebo models from git@github.com:osrf/gazebo_models.git and copying in ~/.gazebo/models/
    #git clone https://github.com/osrf/gazebo_models 
    git clone git@github.com:osrf/gazebo_models.git
    mv ./gazebo_models/ ~/.gazebo/models/
fi    

# You can also find some nice gazebo worlds here:
# https://dev.px4.io/v1.11_noredirect/en/simulation/gazebo_worlds.html 
# https://github.com/PX4/PX4-SITL_gazebo-classic 
# In order to let gazebo find the downloaded models therein, you can export: 
# `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"<path-to-repo>/PX4-SITL_gazebo-classic/models"`
if [ ! -d PX4-SITL_gazebo-classic ]; then 
    echo cloning gazebo models from git@github.com:PX4/PX4-SITL_gazebo-classic.git and exporting their path 
    #git clone https://github.com/PX4/PX4-SITL_gazebo-classic 
    git clone git@github.com:PX4/PX4-SITL_gazebo-classic.git
    MODELS_DIR=`pwd`
    echo exporting additional gazebo models folder: $GAZEBO_MODEL_PATH
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"$MODELS_DIR/PX4-SITL_gazebo-classic/models"
fi 

cd $STARTING_DIR
