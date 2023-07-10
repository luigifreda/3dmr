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

## Localization 

At the present time, I removed gmapping (which is for 2D). Let's focus on the planning aspect. 

## Exploration

Currently, simulations are performing exploration with 3dmr method. 

RVIZ file is updated for new vizualizations (nodes, status, octomaps...)

To run the simulation:
```
$ rosrun expl_planner sim_launcher_exploration
```

## Issues

### Volumetric mapping 

Given the fact that RGBD scans are acquired at almost 4 Hz (in TRADR, assembled laser scans were published at 0.3 Hz circa), volumetric mapping is very very loaded (I decreased the tf cache to 10 in order to make it work). This fact slows down everything. We should consider other possible volumetric representations which are more suitable for RGBD scans such as voxblox. We'll talk about that later. 

#### Notes


