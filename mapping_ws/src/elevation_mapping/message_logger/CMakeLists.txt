cmake_minimum_required (VERSION 2.8)

project(message_logger)

# Find Catkin
find_package(catkin REQUIRED)

include(CMakeDependentOption)
cmake_dependent_option(MELO_USE_COUT "Use std::cout" OFF
                       "DEFINED ENV{ROS_DISTRO}" ON)

# Configure header file before catkin_package
catkin_destinations()
configure_file(
  include/message_logger/log/log_messages_backend_config.hpp.in
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/log/log_messages_backend_config.hpp @ONLY
)

if(NOT MELO_USE_COUT)
  message(STATUS "Building message_logger using ros.")
  catkin_package(
    INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    CFG_EXTRAS message_logger.cmake
  )
else()
  message(STATUS "Building message_logger using std::cout.")
  catkin_package(
    INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    CFG_EXTRAS message_logger.cmake
  )
endif()

if(DEFINED MELO_FUNCTION_PRINTS)
  add_definitions(-DMELO_FUNCTION_PRINTS)
endif()

add_definitions(-std=c++11 -fPIC -Wall)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/time/Time.cpp
  src/time/TimeStd.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.in" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/EmptyTests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries( test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      gtest_main
    )
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()
