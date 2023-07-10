
# Install 3DMR

Please, read the main **[README.md](./README.md)** before starting with this file.

## Prerequisites

Our framework works under **Ubuntu 20.04** and **ROS noetic**. In any case, it should be easy to compile it under other platforms. 


## Quick start

If you didn't use the `--recursive` option when git cloning, then start by running:
`git submodule update --init --recursive`.

Starting from the root folder of this repo, run the following commands: 
* install V-REP, Gazebo and ROS dependencies
`$ ./install.sh`
* compile all the workspaces 
`$ ./build_all.sh`
* you can use the following command to clean the workspaces 
`$ ./clean_all.sh `
* source the workspaces by using
`$ source source_all.bash`

Now, you're ready to test patrolling, exploration and navigation systems (see the main **[README.md](./README.md)** file). 

You can find below some manual installation steps. Please, consider that you can find the same commands wrapped for you inside the above installation/build scripts. 

## Manually install necessary tools and ROS dependencies

We discourage a manual installation procedure. However, if you need it, below you can find some required installation steps. Please, consider that these are wrapped for you inside the **install scripts** mentioned above. 

* install catkin tools following the instructions on this page 
http://catkin-tools.readthedocs.io/en/latest/installing.html

* install V-REP (see the section below)

* install necessary ROS dependencies 
```
$ rosdep install --from-paths mapping_ws/src --ignore-src -r
$ rosdep install --from-paths patrolling_ws/src --ignore-src -r
```

* these are some required packages 
```
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-$ROS_DISTRO-octomap-mapping ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-octomap-server
sudo apt-get install -y ros-$ROS_DISTRO-move-base-msgs 
sudo apt-get install -y ros-$ROS_DISTRO-move-base
sudo apt-get install -y ros-$ROS_DISTRO-tf2-geometry-msgs 
sudo apt-get install -y ros-$ROS_DISTRO-tf2
sudo apt-get install -y ros-$ROS_DISTRO-joy
sudo apt-get install -y ros-$ROS_DISTRO-navigation
sudo apt-get install -y ros-$ROS_DISTRO-gazebo-dev
sudo apt-get install -y sox
sudo apt-get install -y doxygen
```

**N.B.** every time you have a failure with a [package-name] you can run
`$ rosdep install [package-name]`  this automatically dowloads and installs system dependancies for the package [package-name] at hand.

## Manual V-REP install

Below you can find some required installation steps. Please, consider that these are wrapped for you inside the installation scripts. 

1. Download version <strike>3.2.2</strike> (tested) from here    
<strike>http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_2_2_64_Linux.tar.gz</strike>    
<strike>http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_3_2_64_Linux.tar.gz</strike>    
http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_5_0_Linux.tar.gz

2. You don't need to compile anything. Just extract the files in your V-REP installation folder and you are ready to execute the main launcher (vrep.sh) from there.

3. Set the environment variable VREP_ROOT_DIR: add in your .bashrc the following line  
`export VREP_ROOT_DIR=<absolute path of your V-REP installation folder (which contains the launcher vrep.sh)>`  
for instance  
<strike>`export VREP_ROOT_DIR=/usr/local/V-REP_PRO_EDU_V3_2_2_64_Linux`</strike>   
<strike>`export VREP_ROOT_DIR=/usr/local/V-REP_PRO_EDU_V3_3_2_64_Linux`</strike>     
`export VREP_ROOT_DIR=/usr/local/V-REP_PRO_EDU_V3_5_0_Linux`

4. Once you have updated and compiled the tradr-simulation stack, you have to copy the lib `patrolling_ws/devel/lib/libv_repExtRos.so` in the
installation folder VREP_ROOT_DIR (NOTE: this lib enables V-REP to get and parse track-velocity command messages)

**Note**: <strike>at present time, V-REP 3.3 does not work with our framework. Please, use V-REP 3.2.2. We will fix the current issues (UGV model scripts and V-REP 3.3) ASAP.</strike> At present time V-REP 3.5 seems to correctly work. In the different scenes, UGV script has been fixed in order to properly work. If you find a scene that does not correctly work, please, check that the contained UGV scripts have the `jaco` arm lines commented (compare with this [script](./patrolling_ws/src/vrep/vrep_ugv_simulation/data/UGV-Script.txt)). 

## Test V-REP installation 

You can test if V-REP is correctly installated by running the following commands: 
`$ cd $VREP_ROOT_DIR`   
`$ sh vrep.sh`   

**Note**: the environment variable should have been set for you by the install script `install.sh` inside  `~/.bashrc`. 

In order to check if the TRADR UGV model is correctly managed, run from the root folder of the 3dpatrolling repo: 
`$ source source_all.bash`   
`$ roslaunch vrep_ugv_simulation vrep_ugv_simulation.launch`    
Once the V-REP main window shows up, press the play button.  
You can then shut down all the nodes by running     
`$ rosrun path_planner kill_vrep_sim`


## Robots do not show up in RVIZ 

If robots do not appear in RVIZ, you can easily solve this issue by running in your shell:     
`$ export LC_NUMERIC="en_US.UTF-8"`    

Now, this command has been hadded in the `source_all.bash` scripts. 
For further details, see [this page](https://answers.ros.org/question/266313/robot-model-not-showing-in-rviz/
).






