This package contains some launch files for testing and applying tf remapping 

* tf_remap_test_bag_plus_mapper.launch     (set your bag filename as input; you need the new laser slam ws in order to test it!)
The /map frame of ugv1 is remapped to /map1
The /map frame of ugv2 is remapped to /map2
The /map frame is still existing and is connected to /map1 and /map2 by using two static transfomations (for obtainig multi-robot localization)
The laser_mappers will correctly connect the odom frame of each robot to /mapi frame (this has been successfully tested by using the above launch file). Have a look at the attached file generated for ugv1.

* tf_remap_test_bag.launch
this can be used to play a robot bag and remap the frames. 
You have to set the filename argument.
Frames are renamed according to the "mappings" argument.
Topics must be explicitely remapped. 
The frames_id conveyed by the topics are not remapped. 

* tf_remap_test_turtlesim.launch
a simple test: this launches the tf remapper and the launch_and_remap_turtlesim.launch in order to remap the frames of the turtle sim

As for tf_remap_test_bag.launch, few things to bear in mind: 
* tf frames are renamed according to the "mappings" argument of the launch file, as expected.
* topics must be explicitly remapped: this can be dealt at node level or at launch file level, 
i.e. by using the <remap> syntax (as we did in the launch file tf_remap_test_bag.launch )
* the frames_id conveyed by the topics are NOT remapped! A possible solution: the nodes should be modified in order to take the managed frame_ids as argument! 
See for instance sim_trajectory_control_ugv1.launch in the package trajectory_control. 
