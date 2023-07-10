# Copyright (C) 2018 INRIA
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

set(CURRENT_FILE_PATH ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

#.rst:
# .. command:: ADD_GTEST_SUITE([GIT_TAG])
#
#    GIT_TAG: the git tag of gtest. This optional argument allows to use a precise version of gtest (not necessarily the last master branch).
#
#    Download and configure gtest.
#    This macro follows the https://github.com/google/googletest/tree/master/googletest#incorporating-into-an-existing-cmake-project instructions. 
#
MACRO(ADD_GTEST_SUITE)
  # Handle optional argument
  set(GTEST_GIT_TAG "master")
  set(extra_macro_args ${ARGN})
  list(LENGTH extra_macro_args num_extra_args)
  if(${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 GTEST_GIT_TAG)
  endif()
  # Download and unpack googletest at configure time
  configure_file(${CURRENT_FILE_PATH}/gtest/CMakeLists.txt.in gtest/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest )
  if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest )
  if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  # Prevent overriding the parent project's compiler/linker
  # settings on Windows
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This defines
  # the gtest and gtest_main targets.
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/gtest/src
                   ${CMAKE_CURRENT_BINARY_DIR}/gtest/build
                   EXCLUDE_FROM_ALL)

  # Force the include directories to be silent with respect to warnings.
  include_directories(SYSTEM "${gtest_SOURCE_DIR}/include")

ENDMACRO(ADD_GTEST_SUITE)
