cmake_minimum_required(VERSION 2.8.3)
project(multiagent_collision_check)

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CUSTOM_C_FLAGS  "-O3 -DNDEBUG -march=native") 
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")
endif()

add_definitions(-DPCL_NO_PRECOMPILE)  # custom for this project 

find_package(catkin REQUIRED COMPONENTS
  octomap_world_expl
  message_generation
)

add_message_files(
  FILES
  Segment.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multiagent_collision_check_lib ${OCTOMAP_LIBRARIES} #${catkin_LIBRARIES}
)
add_library(multiagent_collision_check_lib src/multiagent_collision_checker.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

target_link_libraries(multiagent_collision_check_lib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
