# Pioneer Workspace

## Simulation with Pioneers 

In order to simulate 3D navigation with pioneer you can launch 
```
$ rosrun vrep_pioneer_simulation sim_launcher_navigation
```
Take a look into the script [sim_launcher_navigation](./pioneer_nav/vrep_pioneer_simulation/scripts/sim_launcher_navigation): if you want to launch a simulation with two pioneers you can by setting 
```
WORLD_NAME=pioneer2 
```
RVIZ is already set in order to host the interface to manage two pioneers as two TRADR robots. 

## Exploration

To run the simulation:
```
$ rosrun expl_planner sim_launcher_exploration
```

## Volumetric mapping 

Given the fact that RGBD scans are acquired at above 10Hz (in TRADR, assembled laser scans were published at the much lower rate of ~0.3 Hz), volumetric mapping is very very loaded and slows down everything. In order to cope with this problem, we use the `scan_space_time_filter` for a proper spatial-time downsampling of the scans. This allows a better workload balance and avoids too much redundancy when processing the scans.
