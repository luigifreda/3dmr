# Path Planning Stack

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
`$ rosrun path_planner sim_launcher_navigation`   

or you can also run:   
`$ source source_all.bash`   
`$ roscd path_planner/scripts`   
`$./sim_launcher_navigation`     


*N.B.*: in the script `sim_launcher_navigation`, you can find some input variables for setting different things, e.g. you can change the V-REP *world* and the *number of robots* (see the variables `WORLD_NAME` and `NUM_ROBOTS`).

 In order to kill all the launched nodes and V-REP, run:   
`$ rosrun path_planner kill_vrep_sim`   
or
`$ ./kill_vrep_sim` (from `path_planner/scripts`)    

---
## Information about the nodes 

### Node: traversability

**Description**: 
provides traversability analysis

**Subscribed topics**:
	/dynjoinpcl: 3d map

**Published topics**:
	/clustered_pcl/wall: subset of map relative to non-traversable parts (walls, obstacles, ...)
	/clustered_pcl/no_wall: subset of map relative to traversable parts (ground, ramp, stairs)
	/clustered_pcl/segmented: ?
	/normals_pcl: point cloud normals
	/normals_markers: point cloud normals marker
	/trav/label: points labeled with semantic information (a priori cost based on the type of traversable part)
	/trav/density: points labeled with local point cloud density
	/trav/roughness: points labeled with local point cloud roughness
	/trav/clearance: points labeled with information about distance to closest obstacle
	/trav/traversability: costmap summarizing the 4 cost components (label, density, roughness, clearance)

**Parameters**:
	ClusteringPcl:
		max_superable_height: 0.2 (maximum height of a surmountable obstacle)
		normal_clustering_thres: 0.1 (?)
		normal_radius_search: 0.3 (normal estimation radius search)
		pcl_cluster_tolerance: 0.3 (?)

---
### Node: pathPlanner

**Description**: 
path planning facility

**Subscribed topics**:
	/trav/traversability: traversability costmap
	/clustered_pcl/wall: (don't know why this guy is here)
	/goal_topic: geometric goal location

**Published topics**:
	/robot_path: geometric path towards goal (if any)


---
### Node: mapping

**Description**: 
aggregates point clouds from laser scanner and remove outdated dynamic obstacles from map

**Subscribed topics**:
	/dynamic_point_cloud: 3d point cloud from laser scanner

**Published topics**:
	/dynjoinpcl: aggregated point cloud
	/normals_marker: debug normals estimation

**Parameters**:
	DynamicJoinPcl:
		global_frame: /map
		laser_frame: /laser
		leaf_size: 0.035 (downsampling leaf size)
		num_subdivisions: 50 (number of subdivisions. higher number -> higher accuracy. beware: too much high number -> out of memory)
	NormalEstimationPcl:
		flatness_curvature_threshold: 0.2
		kernel_type: 0
		laser_frame: /laser
		num_threads: 4
		radius: 0.2
		smoothing: 1.0

