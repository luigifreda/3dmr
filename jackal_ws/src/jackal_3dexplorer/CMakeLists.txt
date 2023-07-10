cmake_minimum_required(VERSION 3.0.2)
project(jackal_3dexplorer)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(
DIRECTORY launch scripts 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
