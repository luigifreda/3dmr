cmake_minimum_required(VERSION 3.0.2)
project(jackal_3dnav)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

# roslaunch_add_file_check(launch/include/amcl.launch)
# roslaunch_add_file_check(launch/amcl_demo.launch)
# roslaunch_add_file_check(launch/include/gmapping.launch)
# roslaunch_add_file_check(launch/gmapping_demo.launch)
# roslaunch_add_file_check(launch/include/move_base.launch)
# roslaunch_add_file_check(launch/odom_navigation_demo.launch)

install(
DIRECTORY launch scripts 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
