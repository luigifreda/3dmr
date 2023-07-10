cmake_minimum_required(VERSION 2.8.3)
project(voxblox_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Wextra)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CUSTOM_C_FLAGS  "-O3 -DNDEBUG -march=native") 
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")
endif()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########

cs_install()
cs_export()
