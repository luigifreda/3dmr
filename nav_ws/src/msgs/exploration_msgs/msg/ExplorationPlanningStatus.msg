uint8 kNone             = 0

std_msgs/Header header
bool success               # result of the exploration planner: false in case a solution was not found for some reason, true otherwise
uint8 status		       # status type: detailed status of the exploration planner 
float64 path_cost	       # cost of the planned path: "< 0" means invalid
float64 gain               # information gain 

