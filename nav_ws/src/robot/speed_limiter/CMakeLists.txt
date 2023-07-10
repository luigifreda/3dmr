cmake_minimum_required(VERSION 2.8.3)
project(speed_limiter)

find_package(catkin REQUIRED)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs
)

#############
## Install ##
#############

install(PROGRAMS
   nodes/speed_limiter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  config/limit_explanations.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)