cmake_minimum_required(VERSION 2.8.3)
project(patrolling3d_sim)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CUSTOM_C_FLAGS "-g3 -ggdb -Wall -pthread")

set(PKG_DEPS
  actionlib
  move_base_msgs  
  nav_msgs
  roscpp
  trajectory_control_msgs
  octomap_msgs
  nifti_robot_driver_msgs
  patrolling_build_graph_msgs
  stamped_msgs
  tf
  path_planner
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#   move_base_msgs#   nav_msgs
# )

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
  INCLUDE_DIRS src
  LIBRARIES 
   PatrolAgent SSIPatrolAgent
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
# include_directories(include)
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(PatrolAgent
  src/PatrolAgent.cpp
  src/graph.cpp
  src/algorithms.cpp
  src/config.cpp
  src/PatrollingMarkerController.cpp
)
target_link_libraries(PatrolAgent ${catkin_LIBRARIES})
add_dependencies(PatrolAgent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
add_dependencies(Conscientious_Reactive  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(INCLUDE_OLD_TARGETS FALSE)
if(INCLUDE_OLD_TARGETS) ########################################################

# N.B. the following nodes belong to the original package patrolling_sim but they are not yet integrated in the new framework!

# Declare a cpp library
add_library(SSIPatrolAgent src/SSIPatrolAgent.cpp)
target_link_libraries(SSIPatrolAgent PatrolAgent ${catkin_LIBRARIES})
add_dependencies(SSIPatrolAgent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
## Declare a cpp executable
add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
add_dependencies(Heuristic_Conscientious_Reactive  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Conscientious_Cognitive PatrolAgent ${catkin_LIBRARIES})
add_dependencies(Conscientious_Cognitive  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(Cyclic src/Cyclic_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Cyclic PatrolAgent ${catkin_LIBRARIES})
add_dependencies(Cyclic  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(MSP src/MSP_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(MSP PatrolAgent ${catkin_LIBRARIES})
add_dependencies(MSP  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(GBS src/GBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(GBS PatrolAgent ${catkin_LIBRARIES})
add_dependencies(GBS  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(SEBS src/SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(SEBS PatrolAgent ${catkin_LIBRARIES})
add_dependencies(SEBS  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(CBLS src/CBLS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(CBLS PatrolAgent ${catkin_LIBRARIES})
add_dependencies(CBLS  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(Random src/Random_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Random PatrolAgent ${catkin_LIBRARIES})
add_dependencies(Random  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTAGreedy PatrolAgent ${catkin_LIBRARIES})
add_dependencies(DTAGreedy  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(DTASSI src/DTASSI_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
add_dependencies(DTASSI  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(DTASSIPart src/DTASSIPart_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
add_dependencies(DTASSIPart  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

endif() ########################################################
 
# Results and Monitor Node:
 
## Declare a cpp executable 
add_executable(monitor src/monitor.cpp src/graph.cpp src/graph_viz.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
## Declare a cpp executable 
add_executable(idlHistogram src/idlHistogram.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(idlHistogram ${catkin_LIBRARIES})
#add_dependencies(idlHistogram  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(patrolling3d_sim_node patrolling3d_sim_generate_messages_cpp)


 



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
# install(TARGETS patrolling3d_sim patrolling3d_sim_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_patrolling3d_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(Conscientious_Reactive src/Conscientious_Reactive.cpp)
#add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive.cpp)
#add_executable(Conscientious_Cognitive src/Conscientious_Cognitive.cpp)
#add_executable(Cyclic src/Cyclic.cpp)
#add_executable(MSP src/MSP.cpp)
#add_executable(GBS src/GBS.cpp)
#add_executable(SEBS src/SEBS.cpp)

