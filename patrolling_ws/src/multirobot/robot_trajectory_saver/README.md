This node implements the following functionalities

* service to store into a file "/my_path/filename.dot" in the form of a graph (boost) the trajectories of the robots
	** rosservice call /robot_trajectory_saver_node/save_robot_trajectories_only "file_path: '/my_path/filename.dot'"  

* servise to get the graph of the robot trajectories in the form of a nav_msgs::Path 
	** rosservice call /robot_trajectory_saver_node/get_robot_trajectories_nav_msgs "{}" (there is no request record into the srv msg please check if last parameter of the command is "{}") 
