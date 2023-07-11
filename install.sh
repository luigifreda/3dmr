#!/usr/bin/env bash


# ====================================================

function print_blue(){
	printf "\033[34;1m"
	printf "$@ \n"
	printf "\033[0m"
}

# ====================================================

print_blue '================================================'
print_blue 'Installing and building 3DMR'
print_blue '================================================'

set -e

# update all submodules 
# If you didn't use the `--recursive` option when cloning, then run:
git submodule update --init --recursive 

sudo apt-get update 

version=$(lsb_release -a 2>&1)

# --------------------------
# install catkin tools 
# --------------------------
# from https://catkin-tools.readthedocs.io/en/latest/installing.html

print_blue 'Installing catkin tools'

# first you must have the ROS repositories which contain the .deb for catkin_tools:
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# once you have added that repository, run these commands to install catkin_tools:
sudo apt-get update
if [[ $version == *"18.04"* ]] ; then
	sudo apt-get install -y python-catkin-tools
else
	sudo apt-get install -y python3-catkin-tools
fi


# --------------------------
# solve problems with ROS melodic  
# --------------------------

if [ ! -d  patrolling_ws/src/msgs ]; then
	mkdir -p patrolling_ws/src/msgs
fi
cd patrolling_ws/src/msgs 
if [ ! -d  brics_actuator ]; then
	echo 'downloading brics_actuator'
	git clone https://github.com/wnowak/brics_actuator
fi 
cd - 

# --------------------------
# install necessary workspace dependencies 
# --------------------------

print_blue 'ROS dependencies'

rosdep update
rosdep install --from-paths mapping_ws/src nav_ws/src patrolling_ws/src exploration_ws/src pioneer_ws/src jackal_ws/src teb_ws/src --ignore-src -y

sudo apt-get install -y cmake-extras     # https://bugs.launchpad.net/ubuntu/+source/googletest/+bug/1644062

if [[ $version == *"18.04"* ]] ; then
	sudo apt-get install -y python-rosinstall python-rospkg
else
	sudo apt-get install -y python3-rosinstall python3-rospkg
fi
sudo apt-get install -y ros-$ROS_DISTRO-octomap-mapping ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-octomap-server
sudo apt-get install -y ros-$ROS_DISTRO-move-base-msgs 
sudo apt-get install -y ros-$ROS_DISTRO-move-base
sudo apt-get install -y ros-$ROS_DISTRO-tf2-geometry-msgs 
sudo apt-get install -y ros-$ROS_DISTRO-tf2
sudo apt-get install -y ros-$ROS_DISTRO-joy
sudo apt-get install -y ros-$ROS_DISTRO-navigation
sudo apt-get install -y ros-$ROS_DISTRO-gazebo-dev
sudo apt-get install -y ros-$ROS_DISTRO-gmapping 
sudo apt-get install -y sox
sudo apt-get install -y doxygen
sudo apt-get install -y screen 
sudo apt-get install -y xterm

sudo apt install -y ros-$ROS_DISTRO-jackal-gazebo
sudo apt install -y ros-$ROS_DISTRO-jackal-*
sudo apt install -y ros-$ROS_DISTRO-velodyne-*
sudo apt install -y ros-$ROS_DISTRO-geodesy
sudo apt install -y ros-$ROS_DISTRO-nmea-*
sudo apt install -y ros-$ROS_DISTRO-pcl-ros
sudo apt install -y ros-$ROS_DISTRO-lms1xx ros-$ROS_DISTRO-pointgrey-camera-description ros-$ROS_DISTRO-twist-mux ros-$ROS_DISTRO-interactive-marker-twist-server

if [[ $version == *"18.04"* ]] ; then
	sudo apt-get install -y libqt4-dev 
else
	sudo apt-get install -y qt5-default 
fi 

sudo apt-get install -y libnss3-dev

sudo apt-get install -y liboctomap-dev
sudo apt-get install -y protobuf-compiler libgoogle-glog-dev
sudo apt-get install -y googletest google-mock 

# compile google-test dev   from https://stackoverflow.com/questions/24295876/cmake-cannot-find-googletest-required-library-in-ubuntu
sudo apt-get install -y libgtest-dev
cd /usr/src/googletest
sudo cmake .
sudo cmake --build . --target install
cd - 

# these are necessary for the qt application with PyQt5
sudo apt-get install -y python3-pip 
pip3 install pyqt5==5.14.0 --user  # issues with newest PyQt5 https://stackoverflow.com/questions/59711301/install-pyqt5-5-14-1-on-linux

if [[ $version == *"18.04"* ]] ; then
	sudo apt-get install -y libcanberra-gtk-module libcanberra-gtk3-module
	sudo apt-get install -y libqt4-dev python-qt4
else
    pip install --upgrade pip
	pip install pydot
    pip install graphviz
	pip install PyQt5
	sudo apt-get install -y qt5-default python3-pyqt5 
fi 

# for running demos from https://github.com/ethz-asl/rotors_simulator
if [ ! -f /.dockerenv ]; then # if we are outside a docker container 
	sudo apt install python3-uinput
	pip install python-uinput
	pip install pygame
	modprobe -i uinput
	sudo addgroup uinput
	sudo adduser $USER uinput
	# from https://tkcheng.wordpress.com/2013/11/11/changing-uinput-device-permission/ 
	if [ ! -f /etc/udev/rules.d/99-input.rules ]; then 
		sudo touch /etc/udev/rules.d/99-input.rules
		echo 'KERNEL=="uinput", GROUP="uinput", MODE:="0660"' | sudo tee -a /etc/udev/rules.d/99-input.rules
		sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger # instead of rebooting 
	fi 
fi 

# --------------------------
# install V-REP
# --------------------------
./install-vrep.sh

# --------------------------
# install Gazebo 
# --------------------------
./install-gazebo.sh


# --------------------------
# Build all
# --------------------------
#./build_all.sh






