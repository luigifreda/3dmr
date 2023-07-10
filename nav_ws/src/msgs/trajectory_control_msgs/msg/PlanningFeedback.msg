std_msgs/Header header
string node   # identifies the node advertising the message: "control" (controller), "planner" (queue planner) or "tool" (rviz waypoints tool)
string task   # the task name (convention: TaskX, X number of the task)
int8 action   # the action performed on the task 
int8 status   # STATUS_FAILURE = -1, STATUS_PLANNING = 0, STATUS_SUCCESS = 1
