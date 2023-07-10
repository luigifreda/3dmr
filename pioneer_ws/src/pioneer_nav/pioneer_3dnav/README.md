# pioneer_3dnav


## Main scripts 

In the folder `scripts`, you can find: 
- `sim_launcher_navigation` 
for launching a path planning simulation with all the required nodes (mapping, traversability analysis, trajectory control)
- `sim_launcher_ugv`
for launching the nodes of a single robot (mapping, path planner, trajectory control): this script is used by `sim_launcher_navigation`
- `save_map` 
for saving the robot map and trajectory (please, use `ugv1` for building and saving maps or trajectories)
- `kill_vrep_sim` or `kill_gazebo_sim`
for killing all the launched nodes and V-REP/gazebo.

## How to run a path planning simulation

Open a new terminal, enter in the root folder of the repo, and run:   
`$ source source_all.bash`   
`$ rosrun pioneer_3dnav sim_launcher_navigation`   

or you can also run:   
`$ source source_all.bash`   
`$ roscd pioneer_3dnav/scripts`   
`$./sim_launcher_navigation`     


*N.B.*: in the script `sim_launcher_navigation`, you can find some input variables for setting different things, e.g. you can change the V-REP *world* and the *number of robots* (see the variables `WORLD_NAME` and `NUM_ROBOTS`).

 In order to kill all the launched nodes and V-REP, run:   
`$ rosrun pioneer_3dnav kill_vrep_sim`   
or
`$ ./kill_vrep_sim` (from `pioneer_3dnav/scripts`)    