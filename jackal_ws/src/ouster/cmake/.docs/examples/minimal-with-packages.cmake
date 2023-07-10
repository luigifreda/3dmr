# Target-based approach should work from CMake 2.8.12 but it should fully work from 3.1
cmake_minimum_required(VERSION 2.8.12)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME jrl-cmakemodules-minimal-working-example)
set(PROJECT_DESCRIPTION "A project description")
set(PROJECT_URL http://jrl-cmakemodules.readthedocs.io)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)

project(${PROJECT_NAME} CXX)

# Add a required dependency
add_project_dependency(MyDependency REQUIRED)

# Another example to show that arguments can be passed down to the underlying find_package call
add_project_dependency(Boost 1.50 REQUIRED COMPONENT timer)

add_library(myLibrary ${MY_SOURCES})
target_link_libraries(myLibrary MyDependency::MyAwesomeLib Boost::timer)

install(TARGETS myLibrary
        EXPORT ${TARGETS_EXPORT_NAME}
        DESTINATION lib)
