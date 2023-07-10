# sender type
uint8 kSenderExplorer = 0
uint8 kSenderGUI      = 1

# A reference coordinate frame and timestamp
Header header

uint8 sender                                # sender type

int32 num_points	                    # number of points
ExplorationPriorityPoint[] priority_points  # list of priority points with actions

