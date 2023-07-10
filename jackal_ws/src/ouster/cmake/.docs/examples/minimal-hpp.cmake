cmake_minimum_required(VERSION 2.8)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME hpp-project-example)
set(PROJECT_DESCRIPTION "A HPP project example")

# hpp.cmake includes base.cmake.
include(cmake/hpp.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Configure the build of your project here
# add_subdirectory(src)
