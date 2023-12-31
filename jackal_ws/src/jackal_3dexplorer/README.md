# jackal_3dexplorer

A set of launch files for launching an exploration with jackal UGVs.

## How to run an exploration 

Open a new terminal, and enter in the root folder of our repo `3DMR` and run:   
`$ source source_all.bash`   
`$ rosrun jackal_3dexplorer sim_launcher_exploration`    

or   
`$ source source_all.bash`     
`$ roscd jackal_3dexplorer/scripts`     
`$ ./sim_launcher_exploration`   
 
At the beginning of the script `sim_launcher_exploration`, you can find some input variables for setting different things, e.g. you can change the gazebo *world* (see the variables `WORLD_NAME`).

 In order to kill all the launched nodes and gazebo, run:   
```
$ rosrun jackal_3dexplorer kill_gazebo_sim
```
(or press the button `Kill all` on the Qt GUI) at the end of each simulation. This is **very important** and does actually guarantee that a proper shutdown of all the launched ROS nodes and gazebo application was performed. 
