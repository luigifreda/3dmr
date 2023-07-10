cmake_minimum_required(VERSION 3.2)

set(PROJECT_NAME jrl-cmakemodules-cpp)
set(PROJECT_VERSION 0.0.0)
set(PROJECT_DESCRIPTION "JRL CMake module - cpp")
set(PROJECT_URL http://jrl-cmakemodules.readthedocs.io)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(../../base.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

set(${PROJECT_NAME}_HEADERS
  include/jrl_cmakemodule/lib.hh)

add_library(jrl_cmakemodule_lib SHARED src/lib.cc)
target_include_directories(jrl_cmakemodule_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS jrl_cmakemodule_lib
        EXPORT ${TARGETS_EXPORT_NAME}
        DESTINATION lib)

add_executable(jrl_cmakemodule_main src/main.cc)
target_link_libraries(jrl_cmakemodule_main jrl_cmakemodule_lib)

install(TARGETS jrl_cmakemodule_main
        DESTINATION main)
