cmake_minimum_required(VERSION 2.8.3)
project(octomap_demux)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(PACKAGE_DEPS 
  actionlib
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
    ${PACKAGE_DEPS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    ${PACKAGE_DEPS}
  DEPENDS 
   # system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

set(PCL_LIBS_DEPS 
    ${pcl_ros_LIBRARIES} 
    ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} src/octomap_demux.cpp)
target_link_libraries(octomap_demux  ${PCL_LIBS_DEPS})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(octomap_demux_node src/octomap_demux_node.cpp)
add_dependencies(octomap_demux_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_demux_node octomap_demux ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBS_DEPS})

add_executable(octomap_demux5_node src/octomap_demux5_node.cpp src/octomap_demux5.cpp)
add_dependencies(octomap_demux5_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_demux5_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBS_DEPS})



