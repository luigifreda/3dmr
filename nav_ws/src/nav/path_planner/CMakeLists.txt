cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

set(PREPARE_RELASE 1) # 1 Release, 0 Debug

# if no option is given, standard is release
#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

    if(PREPARE_RELASE )
        message(STATUS "${PROJECT_NAME} build type: Release *********************************")
        set(CMAKE_BUILD_TYPE Release)
        set(CUSTOM_C_FLAGS "-O3 -pthread -DNDEBUG -march=native")
    else()
        message(STATUS "${PROJECT_NAME} build type: Debug *********************************")
        set(CMAKE_BUILD_TYPE Debug)
        #message(STATUS "CUSTOM_CXX_FLAGS_DEBUG: ${CUSTOM_CXX_FLAGS_DEBUG}$")
        set(CUSTOM_C_FLAGS "-g3 -ggdb -Wall -pthread")
    endif()

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")

#endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
endif()

set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add OpenMP flags 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


set(PATH_PLANNER_PKG_DEPS
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  laser_geometry
  nav_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  std_msgs
  tf
  trajectory_control_msgs
  actionlib
  actionlib_msgs
  trajectory_control_msgs
  robot_trajectory_saver_msgs
  wireless_network_msgs
  networkanalysis_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ${PATH_PLANNER_PKG_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")
#include_directories(${EIGEN3_INCLUDE_DIR})

#find_package(PkgConfig)
#pkg_search_module(Eigen3 REQUIRED eigen3)​

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

add_definitions(-DPCL_NO_PRECOMPILE)  # custom for this project 

message(STATUS "==============================================================") 
message(STATUS "Found PCL in: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL libs: ${PCL_LIBRARIES}") 
message(STATUS "pcl_ros libs: ${pcl_ros_LIBRARIES}") 
message(STATUS "==============================================================") 


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

generate_dynamic_reconfigure_options(
  cfg/ClusterPcl.cfg
  cfg/DynamicJoinPcl.cfg
  cfg/NormalEstimationPcl.cfg
  cfg/TravAnalyzer.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    normalestimation clusterpcl conversionpcl travanalyzerpcl pathplanning pathplanningutils #dynamicjoinpcl
  CATKIN_DEPENDS 
    ${PATH_PLANNER_PKG_DEPS}
  DEPENDS 
    #system_lib
    Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
    #cfg/cpp/path_planner
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

#message(FATAL_ERROR "CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}")

link_directories(
    ${PCL_LIBRARY_DIRS}
)

set(PCL_LIBS_DEPS 
    ${pcl_ros_LIBRARIES} 
    ${PCL_LIBRARIES}
)


## Declare a C++ library
# add_library(path_planner
#   src/${PROJECT_NAME}/path_planner.cpp
# )

add_library(pathplanningutils src/Transform.cpp src/Exception.cpp)
add_library(normalestimation src/NormalEstimationPcl.cpp)
add_library(dynamicjoinpcl src/DynamicJoinPcl.cpp)
add_library(clusterpcl src/ClusterPcl.cpp)
add_library(conversionpcl src/ConversionPcl.cpp)
add_library(travanalyzerpcl src/TravAnalyzer.cpp)
add_library(pathplanning src/PathPlanner.cpp src/PathPlannerManager.cpp src/MarkerController.cpp src/CostFunction.cpp)
#add_library(marker src/MarkerController.cpp)  


target_link_libraries(normalestimation  ${PCL_LIBS_DEPS})
target_link_libraries(dynamicjoinpcl  ${PCL_LIBS_DEPS})
target_link_libraries(clusterpcl  ${PCL_LIBS_DEPS})
target_link_libraries(conversionpcl ${PCL_LIBS_DEPS})
target_link_libraries(travanalyzerpcl  ${PCL_LIBS_DEPS})
target_link_libraries(pathplanning  ${PCL_LIBS_DEPS} travanalyzerpcl)
#target_link_libraries(marker  ${PCL_LIBS_DEPS})  


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(pathplanningutils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(normalestimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(dynamicjoinpcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(clusterpcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(conversionpcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(travanalyzerpcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(pathplanning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Declare a C++ executable
# add_executable(path_planner_node src/path_planner_node.cpp)

add_executable(path_planner src/path_planner.cpp)

add_executable(path_planner_manager src/path_planner_manager.cpp)

#add_executable(mapping src/mapping.cpp)
add_executable(traversability src/traversability.cpp)
add_executable(compute_normals src/compute_normals.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_planner  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(path_planner_manager  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#add_dependencies(mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(traversability ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(compute_normals ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
# target_link_libraries(path_planner_node
#   ${catkin_LIBRARIES}
# )

#target_link_libraries(path_planner  marker  pathplanning ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})
target_link_libraries(path_planner pathplanning travanalyzerpcl pathplanningutils ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})
target_link_libraries(path_planner_manager pathplanning travanalyzerpcl pathplanningutils ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})
#target_link_libraries(mapping conversionpcl dynamicjoinpcl normalestimation  ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})
target_link_libraries(traversability clusterpcl  conversionpcl travanalyzerpcl pathplanningutils ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})
target_link_libraries(compute_normals conversionpcl normalestimation pathplanningutils ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})


set(QUEUE_PLANNER_DIR src/queue_planner)
if(EXISTS ${PROJECT_SOURCE_DIR}/${QUEUE_PLANNER_DIR} )
    add_subdirectory(${QUEUE_PLANNER_DIR})
endif()



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS path_planner path_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_path_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


