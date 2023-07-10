cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME ouster-gazebo-simulation)
set(PROJECT_DESCRIPTION "package for the simulation of the Ouster OS1-64 with ros and gazebo.")
set(PROJECT_URL "https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

# JRL-cmakemodule setup
include(cmake/base.cmake)

# Disable -Werror on Unix for now
set(CXX_DISABLE_WERROR ON)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_subdirectory(ouster_gazebo_plugins)
add_subdirectory(ouster_os1_64)
