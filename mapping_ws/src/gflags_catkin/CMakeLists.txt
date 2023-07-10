cmake_minimum_required(VERSION 2.8.3)
project(gflags_catkin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(
  gflags_src
  URL https://github.com/gflags/gflags/archive/v2.1.2.zip
  URL_MD5 5cb0a1b38740ed596edb7f86cd5b3bd8
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cd ../gflags_src &&
     cmake . 
       -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
       -DBUILD_SHARED_LIBS:BOOL=true
       -DGFLAGS_NAMESPACE:STRING=google
       -DCMAKE_BUILD_TYPE:STRING=Release
  BUILD_COMMAND cd ../gflags_src && make -j 8
  INSTALL_COMMAND cd ../gflags_src && make install -j 8
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} gflags_src)
target_link_libraries(${PROJECT_NAME} 
  ${CATKIN_DEVEL_PREFIX}/lib/libgflags${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS gflags-extras.cmake)
