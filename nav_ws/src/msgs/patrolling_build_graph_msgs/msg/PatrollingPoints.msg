# A reference coordinate frame and timestamp
Header header

# 3D spatial graph

# general information
uint32   num_nodes  # number of points of interest

# lists (i-th item of each list contains information concerning the i-th node)
uint32[] node_id                     # node id
geometry_msgs/Point[] node_position  # node position
float32[] node_priority              # priority of a node

