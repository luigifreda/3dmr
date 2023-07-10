cmake_minimum_required(VERSION 2.8.3)
project(nifti_robot_description)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rosgraph
  rospack
  rospy
  xacro
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  cfg/RobotParts.cfg
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure rosgraph rospack rospy xacro
)

###########
## Build ##
###########


#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    nodes/dynamic_robot_model_server.py
    scripts/print_robot_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)

install(FILES
  meshes/antenna.dae
  meshes/battery.dae
  meshes/body.dae
  meshes/bogie.dae
  meshes/flipper.dae
  meshes/laser.dae
  meshes/omnicam.dae
  meshes/top_box.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(FILES
  meshes/collide/antenna.dae
  meshes/collide/battery.dae
  meshes/collide/body.dae
  meshes/collide/bogie.dae
  meshes/collide/flipper.dae
  meshes/collide/laser.dae
  meshes/collide/omnicam.dae
  meshes/collide/top_box.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/collide
)
install(FILES
  textures/antenna_color.png
  textures/body_color.png
  textures/bogie_color.png
  textures/flipper_color.png
  textures/laser_color.png
  textures/omnicam_color.png
  textures/top_box_color.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/textures
)
install(FILES
  launch/display.launch
  launch/load_robot_model.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  old_model/nifti_robot.urdf
  old_model/body.dae
  old_model/body_color.png
  old_model/bogie.dae
  old_model/bogie_color.png
  old_model/flipper.dae
  old_model/flipper_color.png
  old_model/laser.dae
  old_model/laser_color.png
  old_model/omnicam.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/old_model
)
