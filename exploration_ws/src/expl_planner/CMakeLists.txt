cmake_minimum_required(VERSION 2.8.3)
project(expl_planner)


# ### setup git submodules in this workspace 
#set(MY_COMMAND sh)
#set(MY_ARG ${CMAKE_CURRENT_SOURCE_DIR}/../setup_git_submodules.sh "${CMAKE_CURRENT_SOURCE_DIR}/..")
#execute_process(COMMAND ${MY_COMMAND} ${MY_ARG} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE RES)

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# force debug 
# set(CMAKE_BUILD_TYPE Debug)
# set(CUSTOM_C_FLAGS "-g3 -ggdb -Wall")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")

# NOTE: These optimizations do NOT properly work - there is still some problem with Eigen and optimizations (indeed, some of the packages did not properly use Eigen with aligned allocators!)
#       Moreover, all the "tightly-coupled" packages must be compiled with the same optimization flags
# if (CMAKE_BUILD_TYPE STREQUAL "Release")
#    set(CUSTOM_C_FLAGS  "-O3 -DNDEBUG -march=native") 
#    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")
# endif()


set(PKG_DEPS
  roscpp
  actionlib
  actionlib_msgs
  std_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  path_planner
  roscpp
  tf
  trajectory_control_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  octomap_world_expl
  multiagent_collision_check
  kdtree
  exploration_msgs
#  rospy
#  roslib
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ${PKG_DEPS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread system)

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

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
#   nav_msgs#   trajectory_control_msgs
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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    expl_planner
  CATKIN_DEPENDS 
    ${PKG_DEPS}
  DEPENDS 
    #system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/${PROJECT_NAME}
    ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

set(PCL_LIBS_DEPS 
    ${pcl_ros_LIBRARIES} 
    ${PCL_LIBRARIES}
)


## Declare a C++ library
add_library(${PROJECT_NAME}
   src/Tree.cpp
   src/RrtTree.cpp
   src/ExplorationPlanner.cpp
   src/ExplorationPlannerManager.cpp
   src/ExplorationMarkerController.cpp
   src/PriorityQueue.cpp
   src/TeamModel.cpp
   src/ExplorationTree.cpp
   src/FrontierTree.cpp
   src/NavigationTree.cpp
   src/NodeSet.cpp
   src/ScanFileManager.cpp
   src/ScanHistoryManager.cpp   
   src/SpaceTimeFilterBase.cpp      
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(expl_planner_node src/expl_planner_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(expl_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(expl_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})


### 

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(expl_router_node src/expl_router_node.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(expl_router_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(expl_router_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBS_DEPS})


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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_exp_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
