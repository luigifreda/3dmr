cmake_minimum_required(VERSION 2.8)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME jrl-cmakemodules-minimal-working-example)
set(PROJECT_DESCRIPTION "A project description")
set(PROJECT_URL http://jrl-cmakemodules.readthedocs.io)

include(cmake/base.cmake)

project(${PROJECT_NAME} CXX)

# Configure the build of your project here
# add_subdirectory(src)