cmake_minimum_required(VERSION 2.8.3)
project(wireless_network_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files( FILES
  WiFi_location.msg
  networkerrors.msg
  networkdelay.msg
  linkutilization.msg
  wirelesslink.msg
  Levels_WiFi_2D.msg
)

add_service_files(
  FILES
  RequestRSS.srv
  RequestRSS_PC.srv
  RequestRSS_PC_Point.srv
  RequestRSS_PC_sim.srv
  RequestRSS_Publish_PC.srv
  Request_GPR_reset.srv
  RequestRSS_Save_PC.srv
  RequestRSS_Load_PC.srv
)

generate_messages( DEPENDENCIES std_msgs sensor_msgs)

catkin_package( CATKIN_DEPENDS message_runtime )


