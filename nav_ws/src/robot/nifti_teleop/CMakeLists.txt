cmake_minimum_required(VERSION 2.8.3)
project(nifti_teleop)

set(MESSAGE_DEPS
  actionlib
  std_msgs
)

set(THIS_PACKAGE_DEPS
)

set(BUILD_ONLY_DEPS
  message_generation
)

set(RUN_ONLY_DEPS
  geometry_msgs
  joy
  nifti_robot_driver_msgs
  rospy
  sensor_msgs
  stamped_msgs
  tf
  topic_tools
  message_runtime
)

find_package(catkin REQUIRED COMPONENTS
  ${THIS_PACKAGE_DEPS} ${BUILD_ONLY_DEPS} ${MESSAGE_DEPS}
)

add_service_files(
  FILES
  Acquire.srv
  Release.srv
)

add_action_files(
  FILES
  Scanning.action
)

generate_messages(DEPENDENCIES ${MESSAGE_DEPS})

catkin_package(
  CATKIN_DEPENDS ${THIS_PACKAGE_DEPS} ${RUN_ONLY_DEPS} ${MESSAGE_DEPS}
)

install(PROGRAMS
  nodes/joy_to_twist.py
  nodes/nifti_no.py
  nodes/nifti_teleop_tank.py
  nodes/priority_wrapper.py
  nodes/laser_rotate.sh
  nodes/nifti_teleop_helper.py
  nodes/nifti_test_flippers.py
  nodes/nifti_mux_ctrl.py
  nodes/nifti_teleop_joy.py
  nodes/nifti_teleop_xbox.py
  nodes/nth_action.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/nodes
)

install(FILES
  launch/default_teleop_joy.yaml
  launch/nifti_diag.launch
  launch/nifti_test_flippers.launch
  launch/tank.launch
  launch/diag.yaml
  launch/nifti_joy.launch
  launch/nifti_ugv_teleop.launch
  launch/test_wrapper.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
