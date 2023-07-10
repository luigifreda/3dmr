cmake_minimum_required(VERSION 2.8.3)
project(volumetric_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin_simple REQUIRED)
# Catkin simple also generates messages!
catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########
cs_install()
cs_export()
