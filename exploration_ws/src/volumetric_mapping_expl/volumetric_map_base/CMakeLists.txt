cmake_minimum_required(VERSION 2.8.3)
project(volumetric_map_base_expl)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CUSTOM_C_FLAGS "-g3 -ggdb -Wall")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")

# if (CMAKE_BUILD_TYPE STREQUAL "Release")
#     set(CUSTOM_C_FLAGS  "-O3 -DNDEBUG -march=native") 
#     set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")
# endif()

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

add_definitions(-DPCL_NO_PRECOMPILE)  # custom for this project 

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/world_base.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
