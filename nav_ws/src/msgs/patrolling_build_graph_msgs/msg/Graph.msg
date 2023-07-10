uint8 kNavCostConversionFloatToUint = 10

# A reference coordinate frame and timestamp
Header header

# 3D spatial graph

# general information
uint32   num_nodes  # graph number of nodes

# lists (i-th item of each list contains information concerning the i-th node)
uint32[] node_id                     # node id
float32[] node_priority              # priority of a node
geometry_msgs/Point[] node_position  # node position
uint32[] num_neighbours              # number of neighbours, should be at most 8

# matrices of size num_nodes x num_nodes in row-major order 
bool[]    adjacency_matrix
string[]  direction_matrix  # a string in {"N","NE","E","SE","S","SW","W","NW"}
float32[] cost_matrix
