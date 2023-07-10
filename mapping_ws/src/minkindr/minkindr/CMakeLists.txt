cmake_minimum_required(VERSION 2.8.3)
project(minkindr)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Threads)

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_add_gtest(${PROJECT_NAME}_tests 
  test/test-main.cc
  test/test-rotation.cc
  test/test-sim3.cc
  test/test-transformation.cc
)
target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES} 
                                            ${CMAKE_THREAD_LIBS_INIT})

# Add all files to show up in QtCreator.
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()

