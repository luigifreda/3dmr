#/bin/bash
rostopic pub /stageGUIRequest std_msgs/String "data: 'screenshot'"  --once
rosparam set /simulation_abort true
rosparam get /simulation_abort 
