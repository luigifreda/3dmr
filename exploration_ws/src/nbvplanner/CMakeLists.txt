cmake_minimum_required(VERSION 2.8.3)
project(nbvplanner)

## Compile as C++14, supported in ROS noetic and newer
set(CMAKE_CXX_STANDARD 14) # required by new PCL
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ### setup git submodules in this workspace 
set(MY_COMMAND sh)
set(MY_ARG ${CMAKE_CURRENT_SOURCE_DIR}/../setup_git_submodules.sh "${CMAKE_CURRENT_SOURCE_DIR}/..")
execute_process(COMMAND ${MY_COMMAND} ${MY_ARG} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE RES)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world_expl
  tf
  kdtree
  multiagent_collision_check
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

add_service_files(
  FILES
  nbvp_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES 
    nbvPlannerLib ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS 
    message_runtime roscpp geometry_msgs visualization_msgs octomap_world_expl tf kdtree multiagent_collision_check
  DEPENDS 
    Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(nbvPlannerLib src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(nbvPlannerLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} nbvplanner_gencpp)

add_executable(nbvPlanner src/nbv_planner_node.cpp src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(nbvPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} nbvplanner_gencpp)

target_link_libraries(nbvPlannerLib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_link_libraries(nbvPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
