std_msgs/Header header
string name                      # task name; a planning task is a vector of task-segments; a task-segment is composed by a start and a goal (two points). 
int32 segment_id                 # 
int32 segment_count              # count number of waypoints inside the task
uint8  type                      # NORMAL=0, CYCLIC = 1
geometry_msgs/Point[] waypoints  # array of waypoints


