cmake_minimum_required(VERSION 2.8.3)
project(doxygen_catkin)

find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS doxygen_catkin.cmake)
catkin_python_setup()

