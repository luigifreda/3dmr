cmake_minimum_required(VERSION 2.8.3)
project(mav_planning_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs message_generation mav_msgs trajectory_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # ROS == Indigo.
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  # ROS > Indigo.
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_message_files(
  FILES
  Point2D.msg
  PointCloudWithPose.msg
  Polygon2D.msg
  PolygonWithHoles.msg
  PolygonWithHolesStamped.msg
  PolynomialSegment4D.msg
  PolynomialTrajectory4D.msg
)

add_service_files(
  FILES
  PlannerService.srv
  PolygonService.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs mav_msgs trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs mav_msgs trajectory_msgs
)
