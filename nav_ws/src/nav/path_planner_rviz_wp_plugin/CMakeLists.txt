cmake_minimum_required(VERSION 2.8.3)
project(path_planner_rviz_wp_plugin)


set(PKG_REQUIRED_COMPONENTS
  class_loader
  geometry_msgs
  nav_msgs
  std_msgs
  message_filters
  trajectory_control_msgs
  patrolling_build_graph_msgs
  path_planner
  rviz
  tf
  dynamic_reconfigure
  exploration_msgs
  wireless_network_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ${PKG_REQUIRED_COMPONENTS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## We also use Ogre
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

## PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost 
find_package(Boost REQUIRED COMPONENTS thread system)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

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
#   geometry_msgs#   ms_trajectory_control_msgs
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS 
        include
    LIBRARIES 
        path_planner_rviz_wp_plugin
    CATKIN_DEPENDS 
        ${PKG_REQUIRED_COMPONENTS}
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
  include
  ${catkin_INCLUDE_DIRS}
)

# Don't forget to include output directory, otherwise
# the UI file won't be wrapped!
include_directories(${CMAKE_CURRENT_BINARY_DIR})


## Declare a C++ library
# add_library(path_planner_rviz_wp_plugin
#   src/${PROJECT_NAME}/path_planner_rviz_wp_plugin.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(path_planner_rviz_wp_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(path_planner_rviz_wp_plugin_node src/path_planner_rviz_wp_plugin_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(path_planner_rviz_wp_plugin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(path_planner_rviz_wp_plugin_node
#   ${catkin_LIBRARIES}
# )




# main headers to be moc'ed
set(project_MOC_HDRS
  include/WaypointsTool.h
)
set(project_MOC_HDRS_SIM1
  include/WaypointsToolSimUgv1.h
)
set(project_MOC_HDRS_SIM2
  include/WaypointsToolSimUgv2.h
)
set(project_MOC_HDRS_SIM3
  include/WaypointsToolSimUgv3.h
)

set(project_MOC_HDRS_SIM_JACKAL1
  include/WaypointsToolSimJackal1.h
)
set(project_MOC_HDRS_SIM_JACKAL2
  include/WaypointsToolSimJackal2.h
)

# main srcs corresponding to main headers in project_MOC_HDRS
set(project_SRCS
  src/WaypointsTool.cpp
)
set(project_SRCS_SIM1
  src/WaypointsToolSimUgv1.cpp
)
set(project_SRCS_SIM2
  src/WaypointsToolSimUgv2.cpp
)
set(project_SRCS_SIM3
  src/WaypointsToolSimUgv3.cpp
)

set(project_SRCS_SIM_JACKAL1
  src/WaypointsToolSimJackal1.cpp
)
set(project_SRCS_SIM_JACKAL2
  src/WaypointsToolSimJackal2.cpp
)

# ui files 
SET(project_UIS	
    ui/AdvancedWidget.ui
    ui/PriorityDialog.ui
    ui/NavigationDialog.ui
)

# headers corresponding to ui files 
set(project_MOC_HDRS_UI
   include/AdvancedWidgetUI.h 
   include/PriorityDialogUI.h
   include/NavigationDialogUI.h
)

# srcs corresponding to ui files 
set(project_SRCS_UI	
    src/AdvancedWidgetUI.cpp
    src/PriorityDialogUI.cpp
    src/NavigationDialogUI.cpp
)

# headers corresponding to common files 
set(project_MOC_HDRS_COMMON
   include/BoxSliderConnector.h 
)

# set resources
set(project_RES resources/icons.qrc)


#===============================================================================

# generate ui*.h files from .ui files by using QT5_WRAP_UI macro
QT5_WRAP_UI(project_UIS_H ${project_UIS})

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
## After this calls, project_MOC_SRCS = moc_Class1.cxx moc_Class2.cxx moc_Class3.cxx.
QT5_WRAP_CPP(project_MOC_SRCS
  #src/drive_widget.h
  #src/imu_display.h
  #include/WaypointsTool.h
  #src/teleop_panel.h
  ${project_MOC_HDRS}
  ${project_MOC_HDRS_UI}
  ${project_MOC_HDRS_COMMON}
)
QT5_WRAP_CPP(project_MOC_SRCS_SIM1
  #src/drive_widget.h
  #src/imu_display.h
  #include/WaypointsToolSimUgv1.h
  #src/teleop_panel.h
  ${project_MOC_HDRS_SIM1}
  ${project_MOC_HDRS_UI}
  ${project_MOC_HDRS_COMMON}
)
QT5_WRAP_CPP(project_MOC_SRCS_SIM2
  #src/drive_widget.h
  #src/imu_display.h
  #include/WaypointsToolSimUgv2.h
  #src/teleop_panel.h
  ${project_MOC_HDRS_SIM2}
  ${project_MOC_HDRS_UI}
  ${project_MOC_HDRS_COMMON}
)
QT5_WRAP_CPP(project_MOC_SRCS_SIM3
  #src/drive_widget.h
  #src/imu_display.h
  #include/WaypointsToolSimUgv3.h
  #src/teleop_panel.h
  ${project_MOC_HDRS_SIM3}
  ${project_MOC_HDRS_UI}
  ${project_MOC_HDRS_COMMON}
)

QT5_WRAP_CPP(project_MOC_SRCS_SIM_JACKAL1
  #src/drive_widget.h
  #src/imu_display.h
  #include/WaypointsToolSimUgv1.h
  #src/teleop_panel.h
  ${project_MOC_HDRS_SIM_JACKAL1}
  ${project_MOC_HDRS_UI}
  ${project_MOC_HDRS_COMMON}
)
QT5_WRAP_CPP(project_MOC_SRCS_SIM_JACKAL2
  #src/drive_widget.h
  #src/imu_display.h
  #include/WaypointsToolSimUgv2.h
  #src/teleop_panel.h
  ${project_MOC_HDRS_SIM_JACKAL2}
  ${project_MOC_HDRS_UI}
  ${project_MOC_HDRS_COMMON}
)

# generate rules for building source files from the resources
QT5_ADD_RESOURCES(project_RES_SRCS ${project_RES})


## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${project_MOC_SRCS}``.
set(SOURCE_FILES
  #src/WaypointsTool.cpp
  ${project_SRCS}
  ${project_MOC_SRCS}
  ${project_SRCS_UI}
  ${project_UIS_H}
  ${project_RES_SRCS}
)

set(SOURCE_FILES_SIM1
  #src/WaypointsToolSimUgv1.cpp
  ${project_SRCS_SIM1}
  ${project_MOC_SRCS_SIM1}
  ${project_SRCS_UI}
  ${project_UIS_H}
  ${project_RES_SRCS}
)

set(SOURCE_FILES_SIM2
  #src/WaypointsToolSimUgv2.cpp
  ${project_SRCS_SIM2}
  ${project_MOC_SRCS_SIM2}
  ${project_SRCS_UI}
  ${project_UIS_H}
  ${project_RES_SRCS}
)

set(SOURCE_FILES_SIM3
  #src/WaypointsToolSimUgv3.cpp
  ${project_SRCS_SIM3}
  ${project_MOC_SRCS_SIM3}
  ${project_SRCS_UI}
  ${project_UIS_H}
  ${project_RES_SRCS}
)

set(SOURCE_FILES_SIM_JACKAL1
  #src/WaypointsToolSimJackal1.cpp
  ${project_SRCS_SIM_JACKAL1}
  ${project_MOC_SRCS_SIM_JACKAL1}
  ${project_SRCS_UI}
  ${project_UIS_H}
  ${project_RES_SRCS}
)

set(SOURCE_FILES_SIM_JACKAL2
  #src/WaypointsToolSimJackal2.cpp
  ${project_SRCS_SIM_JACKAL2}
  ${project_MOC_SRCS_SIM_JACKAL2}
  ${project_SRCS_UI}
  ${project_UIS_H}
  ${project_RES_SRCS}
)

## Set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

set(PROJECT_NAME_SIM1 "${PROJECT_NAME}_SIM1")
add_library(${PROJECT_NAME_SIM1} ${SOURCE_FILES_SIM1})

set(PROJECT_NAME_SIM2 "${PROJECT_NAME}_SIM2")
add_library(${PROJECT_NAME_SIM2} ${SOURCE_FILES_SIM2})

set(PROJECT_NAME_SIM3 "${PROJECT_NAME}_SIM3")
add_library(${PROJECT_NAME_SIM3} ${SOURCE_FILES_SIM3})


set(PROJECT_NAME_SIM_JACKAL1 "${PROJECT_NAME}_SIM_JACKAL1")
add_library(${PROJECT_NAME_SIM_JACKAL1} ${SOURCE_FILES_SIM_JACKAL1})

set(PROJECT_NAME_SIM_JACKAL2 "${PROJECT_NAME}_SIM_JACKAL2")
add_library(${PROJECT_NAME_SIM_JACKAL2} ${SOURCE_FILES_SIM_JACKAL2})


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME_SIM1} ${${PROJECT_NAME_SIM1}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME_SIM2} ${${PROJECT_NAME_SIM2}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME_SIM3} ${${PROJECT_NAME_SIM3}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME_SIM_JACKAL1} ${${PROJECT_NAME_SIM_JACKAL1}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME_SIM_JACKAL2} ${${PROJECT_NAME_SIM_JACKAL2}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME_SIM1} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME_SIM2} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME_SIM3} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME_SIM_JACKAL1} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME_SIM_JACKAL2} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL



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
# install(TARGETS path_planner_rviz_wp_plugin path_planner_rviz_wp_plugin_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_path_planner_rviz_wp_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


