cmake_minimum_required(VERSION 2.8)
project(odom2tf)

find_package(catkin REQUIRED nav_msgs roscpp tf)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(odom2tf src/odom2tf.cpp)
target_link_libraries(odom2tf ${catkin_LIBRARIES})
