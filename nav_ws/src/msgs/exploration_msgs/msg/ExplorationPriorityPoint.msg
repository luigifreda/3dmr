# action types
uint8 kActionAdd       = 0
uint8 kActionRemove    = 1  
uint8 kActionRemoveAll = 2

# A reference coordinate frame and timestamp
Header header

uint32 id                     # point id
geometry_msgs/Point position  # position
float32 priority              # priority 
uint8 action		      # action type


