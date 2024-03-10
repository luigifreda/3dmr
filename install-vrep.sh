#!/usr/bin/env bash

# you can use this script to install V-REP 

# ====================================================

set -x

function print_blue(){
	printf "\033[34;1m"
	printf "$@ \n"
	printf "\033[0m"
}

function gdrive_download () {
  #CONFIRM=$(wget --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate "https://docs.google.com/uc?export=download&id=$1" -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')
  #wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$CONFIRM&id=$1" -O $2
  #rm -rf /tmp/cookies.txt
  gdown https://drive.google.com/uc?id=$1
}

#sudo_replace_or_add_string_in_file <search_string> <replace_string> <filename>
function sudo_replace_or_add_string_in_file(){
    if sudo grep -q "$1" $3; then
        if [[ $1 != "" && $2 != "" ]]; then
            echo replacing $1 with $2 in file $3
            sudo sed -i "s/$1/$2/" $3
        fi
    else
        sudo bash -c "echo -e "\n$2" >> $3"
    fi
}

# ====================================================

print_blue '================================================'
print_blue 'Installing V-REP'
print_blue '================================================'

set -e
set -x 

pip install gdown
export PATH=$HOME/.local/bin:$PATH

#FILE=V-REP_PRO_EDU_V3_2_2_64_Linux  # not available any more on coppelia website 
#FILE=V-REP_PRO_EDU_V3_3_2_64_Linux # not available any more on coppelia website 
FILE=V-REP_PRO_EDU_V3_5_0_Linux

URL_COPPELIA=http://coppeliarobotics.com/files/$FILE".tar.gz"
URL_MY_DRIVE_ID="1WIQuVPRsyhX851OL8e21hZMTrqCbX8KN"
# https://drive.google.com/file/d/1WIQuVPRsyhX851OL8e21hZMTrqCbX8KN/view?usp=sharing

DOWNLOAD_FROM_COPPELIA=1

DEST_DIR=/usr/local

# =========================================

STARTING_DIR=`pwd`

if [ ! -d temp ]; then
	mkdir temp
fi
cd temp 

# check if url exists 

if wget $URL_COPPELIA >/dev/null 2>&1 ; then
	echo Url : $URL_COPPELIA exists... downloading from Coppelia
else
	echo Url : $URL_COPPELIA doesnt exists anymore... using stored archive in drive
	DOWNLOAD_FROM_COPPELIA=0
fi

# download the file 
if [ ! -f $FILE".tar.gz" ]; then
	if [ $DOWNLOAD_FROM_COPPELIA -eq 1 ]; then 
		echo 'downloading from Coppelia'
		wget $URL_COPPELIA 
	else
		echo 'downloading from drive'
		gdrive_download $URL_MY_DRIVE_ID $FILE".tar.gz"
	fi	
fi

# extract the file
if [ ! -d $FILE ]; then
	tar xvzf $FILE".tar.gz"
fi

# copy extracted dir in destination dir
if [ ! -d $DEST_DIR/$FILE ]; then
	#sudo mv $FILE $DEST_DIR 
	sudo cp -R $FILE $DEST_DIR 

	# from https://forum.coppeliarobotics.com/viewtopic.php?t=3357
	# in order to solve a strange crash 
	sudo chmod -R a+rwX /usr/local/$FILE	

	# change font size for high resolution 
	# from https://forum.coppeliarobotics.com/viewtopic.php?t=6992
	sudo_replace_or_add_string_in_file "highResDisplay = -1" "highResDisplay = 1" "/usr/local/$FILE/system/usrset.txt"
fi

# set VREP_ROOT_DIR env var in ~/.bashrc
echo "export VREP_ROOT_DIR=/usr/local/$FILE" >> ~/.bashrc


cd $STARTING_DIR

#rm -R temp 




