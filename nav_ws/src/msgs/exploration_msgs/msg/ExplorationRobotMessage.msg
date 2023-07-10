uint8 kNone            = 0
uint8 kReached         = 1
uint8 kPlanned         = 2  
uint8 kSelected        = 3 
uint8 kAborted         = 4
uint8 kPosition        = 5            # to share own position 
uint8 kNoInformation   = 6
uint8 kCompleted       = 7

std_msgs/Header header
uint8 robot_id                   # robot id
uint8 action			 # action type
geometry_msgs/Point goal         # the goal
geometry_msgs/Point position     # the goal
nav_msgs/Path path               # the exploration path, as computed by the exploration planner over the search tree
float64 path_cost	         # the current/remaining path length towards the goal ("< 0" means invalid) as computed by the path planner 
