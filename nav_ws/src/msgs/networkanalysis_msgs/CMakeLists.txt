cmake_minimum_required(VERSION 2.8.3)
project(networkanalysis_msgs)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

## System dependencies are found with CMake's conventions




# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   wirelesslink.msg
   linkutilization.msg
   networkerrors.msg
   networkdelay.msg
 )

# Generate services in the 'srv' folder
add_service_files(
   FILES
   pingservice.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action   
   FILES
   pingaction.action
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs actionlib_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS message_runtime actionlib_msgs
)

###########
## Build ##
###########



#############
## Install ##
#############


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_networkanalysis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

