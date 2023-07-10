cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CUSTOM_C_FLAGS  "-O3 -DNDEBUG -march=native") 
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")
endif()

find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/octomap_world.cc
  src/octomap_manager.cc
)

############
# BINARIES #
############
cs_add_executable(octomap_manager
  src/octomap_manager_node.cc
)
target_link_libraries(octomap_manager ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
