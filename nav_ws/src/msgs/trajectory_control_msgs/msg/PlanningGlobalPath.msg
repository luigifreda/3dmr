std_msgs/Header header
string name                      # task name; a planning task is a vector of task-segments; a task-segment is composed by a start and a goal (two points). 
int32 task_id
uint8  type                      # NORMAL=0, CYCLIC = 1
geometry_msgs/Point[] waypoints  # array of N input waypoints
int32[] waypoint_path_idxs       # array of the N indexes identifying the waypoints in the assembled path (i.e. the next msg field) 
nav_msgs/Path path               # the computed path which passes through the given waypoints 


