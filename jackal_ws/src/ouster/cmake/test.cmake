# Copyright (C) 2008-2020 LAAS-CNRS, JRL AIST-CNRS, INRIA.
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

#.rst:
# .. variable:: DISABLE_TESTS
#    :deprecated:
#
#   Boolean variable to configure unit test compilation declared with
#   :command:`ADD_UNIT_TEST`.
#
#   A target *build_tests* is added to compile the unit-tests.
#   In all cases, ``make all && make test`` compiles and runs the unit-tests.
#
#   * if ``OFF`` (default), the unit-tests are compiled with target *all*,
#     as usual.
#   * if ``ON``, a unit-test called *ctest_build_tests* is added.
#     It is equivalent to the command ``make build_tests``.
#     All unit-test added with :command:`ADD_UNIT_TEST` will be executed
#     after unit-test *ctest_build_tests* completed.
#
#     Thus, the unit-tests are not compiled with target *all* but with target *test*.
#     unit-test  is added and all tests added with
IF(DEFINED DISABLE_TESTS)
  MESSAGE(AUTHOR_WARNING "DISABLE_TESTS is deprecated. Use BUILD_TESTING instead.")
  IF(DISABLE_TESTS)
    SET(BUILD_TESTING OFF CACHE BOOL "")
  ELSE()
    SET(BUILD_TESTING ON CACHE BOOL "")
  ENDIF()
ENDIF(DEFINED DISABLE_TESTS)

IF(NOT TARGET build_tests)
  ADD_CUSTOM_TARGET(build_tests)
ENDIF()

IF(NOT DEFINED ctest_build_tests_exists)
  SET_PROPERTY(GLOBAL PROPERTY ctest_build_tests_exists OFF)
ENDIF(NOT DEFINED ctest_build_tests_exists)

#.rst:
# .. command:: CREATE_CTEST_BUILD_TESTS_TARGET
#
#    Create target ctest_build_tests if does not exist yet.
#
MACRO(CREATE_CTEST_BUILD_TESTS_TARGET)
  GET_PROPERTY(ctest_build_tests_exists_value GLOBAL PROPERTY ctest_build_tests_exists)
  IF(NOT BUILD_TESTING)
    IF(NOT ctest_build_tests_exists_value)
      ADD_TEST(ctest_build_tests "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target build_tests -- $ENV{MAKEFLAGS})
      SET_PROPERTY(GLOBAL PROPERTY ctest_build_tests_exists ON)
    ENDIF(NOT ctest_build_tests_exists_value)
  ENDIF(NOT BUILD_TESTING)
ENDMACRO(CREATE_CTEST_BUILD_TESTS_TARGET)

#.rst:
# .. command:: ADD_UNIT_TEST (NAME SOURCE [SOURCE ...])
#
#   The behaviour of this function depends on :variable:`BUILD_TESTING` option.
#
MACRO(ADD_UNIT_TEST NAME)
  CREATE_CTEST_BUILD_TESTS_TARGET()

  IF(NOT BUILD_TESTING)
    ADD_EXECUTABLE(${NAME} EXCLUDE_FROM_ALL ${ARGN})
  ELSE(NOT BUILD_TESTING)
    ADD_EXECUTABLE(${NAME} ${ARGN})
  ENDIF(NOT BUILD_TESTING)

  ADD_DEPENDENCIES(build_tests ${NAME})

  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})
  # Support definition of DYLD_LIBRARY_PATH for OSX systems
  IF(APPLE)
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH};DYLD_LIBRARY_PATH=$ENV{DYLD_LIBRARY_PATH}")
  ENDIF(APPLE)

  IF(NOT BUILD_TESTING)
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES DEPENDS ctest_build_tests)
  ENDIF(NOT BUILD_TESTING)
ENDMACRO(ADD_UNIT_TEST NAME SOURCE)

#.rst:
# .. command:: ADD_PYTHON_UNIT_TEST (NAME SOURCE [MODULES...])
#
#   Add a test called `NAME` that runs an equivalent of ``python ${SOURCE}``,
#   optionnaly with a `PYTHONPATH` set to `CMAKE_BINARY_DIR/MODULE_PATH` for each MODULES
#   `SOURCE` is relative to `PROJECT_SOURCE_DIR`
#
#   .. note:: :command:`FINDPYTHON` should have been called first.
#
MACRO(ADD_PYTHON_UNIT_TEST NAME SOURCE)
  ADD_TEST(NAME ${NAME} COMMAND ${PYTHON_EXECUTABLE} "${PROJECT_SOURCE_DIR}/${SOURCE}")
  SET(PYTHONPATH)

  SET(MODULES "${ARGN}")  # ARGN is not a variable
  FOREACH(MODULE_PATH IN LISTS MODULES)
    LIST(APPEND PYTHONPATH "${CMAKE_BINARY_DIR}/${MODULE_PATH}")
    IF(CMAKE_GENERATOR MATCHES "Visual Studio")
      LIST(APPEND PYTHONPATH "${CMAKE_BINARY_DIR}/${MODULE_PATH}/$<CONFIG>")
    ENDIF(CMAKE_GENERATOR MATCHES "Visual Studio")
  ENDFOREACH(MODULE_PATH IN LISTS MODULES)

  IF(DEFINED ENV{PYTHONPATH})
    LIST(APPEND PYTHONPATH "$ENV{PYTHONPATH}")
  ENDIF(DEFINED ENV{PYTHONPATH})

  # get path separator to join those paths
  EXECUTE_PROCESS(COMMAND
      "${PYTHON_EXECUTABLE}" "-c" "import os; print(os.pathsep)"
      OUTPUT_VARIABLE PATHSEP
      OUTPUT_STRIP_TRAILING_WHITESPACE)

  IF(WIN32)
    STRING(REPLACE ";" ":" PYTHONPATH_STR "${PYTHONPATH}")
  ELSE(WIN32)
    STRING(REPLACE ";" "${PATHSEP}" PYTHONPATH_STR "${PYTHONPATH}")
  ENDIF(WIN32)
  SET(ENV_VARIABLES "PYTHONPATH=${PYTHONPATH_STR}")
  IF(APPLE)
    LIST(APPEND ENV_VARIABLES "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}")
    LIST(APPEND ENV_VARIABLES "DYLD_LIBRARY_PATH=$ENV{DYLD_LIBRARY_PATH}")
  ENDIF(APPLE)
  SET_TESTS_PROPERTIES(${NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
ENDMACRO(ADD_PYTHON_UNIT_TEST NAME SOURCE)

# DEFINE_UNIT_TEST(NAME LIB)
# ----------------------
#
# Compile a program and add it as a test
#
MACRO(DEFINE_UNIT_TEST NAME LIB)
  ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} ${PUBLIC_KEYWORD} ${LIB})
ENDMACRO(DEFINE_UNIT_TEST)
