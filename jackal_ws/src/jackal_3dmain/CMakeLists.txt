cmake_minimum_required(VERSION 2.8.3)
project(jackal_3dmain)

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS 
  #include
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
)
