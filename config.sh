#!/usr/bin/env bash

# here you can load some common env variables 

MR3D_USE_SCREEN=1  # 1 use screen for running ros commands in the scripts; 0 use xterm 

# don't change the order 
LIST_OF_WSS=( \
"$MR3D_HOME/teb_ws" \
"$MR3D_HOME/mapping_ws" \
"$MR3D_HOME/nav_ws" \
"$MR3D_HOME/patrolling_ws" \
"$MR3D_HOME/exploration_ws" \
"$MR3D_HOME/pioneer_ws" \
"$MR3D_HOME/jackal_ws" \
)

########################################################################################################

if [[ -z "${MR3D_HOME}" ]]; then
    echo "ERROR: missing env var MR3D_HOME"
    echo "please, source source_all.bash in the main 3DMR folder"
    exit 
fi

########################################################################################################

SOURCE_FILE=$MR3D_HOME/source_all.bash

# check if log folder is created 
LOG_FOLDER=$HOME/.ros/3dmr/logs
mkdir -p -- "$LOG_FOLDER"

########################################################################################################

# set the locale
export LC_ALL=en_US.UTF-8
export LC_NUMERIC=en_US.UTF-8
export LANG=en_US.UTF-8

if [ -z "$XDG_RUNTIME_DIR" ]; then
    export XDG_RUNTIME_DIR=/tmp/runtime-$USER
    mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
fi

########################################################################################################

function open_screen(){
session_name=$1
command_string=${@:2}
#screen -dmS session_name -Lc file_init  
screen -dmS $session_name -L  
screen -S $session_name -L -X stuff $"source $SOURCE_FILE; $command_string \n"   
}

function open_xterm(){
session_name=$1
command_string=${@:2}
xterm -T $session_name $XTERM_OPTIONS -e "$command_string; bash" &    
}
# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm $XTERM_OPTIONS -e "<you_command>; bash" &

# usage: 
# open_term "NAME_SESSION" "COMMAND_STRING"
# N.B.: "NAME_SESSION" must be a single word without spaces 
function open_term(){
    if [ $MR3D_USE_SCREEN -eq 1 ]; then
        open_screen $@
    else
        open_xterm $@
    fi
}

########################################################################################################
