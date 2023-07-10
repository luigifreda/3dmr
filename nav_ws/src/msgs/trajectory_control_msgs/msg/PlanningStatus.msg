uint8 kNone             = 0
uint8 kNotReady         = 1  
uint8 kInputFailure     = 2 
uint8 kFailure          = 3
uint8 kSuccess          = 4
uint8 kArrived          = 5
uint8 kAborted          = 6
uint8 kTransformFailure = 7
uint8 kFirstSuccess     = 8

std_msgs/Header header
bool success                   # result of the path planner: false in case a solution was not found for some reason, true otherwise
uint8 status		       # status type: detailed status of the path planner (use one of the constants above )
float64 path_cost	       # cost of the planned path: "< 0" means invalid


