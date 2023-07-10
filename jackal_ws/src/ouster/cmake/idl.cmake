# Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
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
# .. command:: OMNIIDL_INCLUDE_DIRECTORIES (DIRECTORIES)
#
#   Set include directories for omniidl
#
#   :param DIRECTORIES: a list of directories to search for idl files.
#
MACRO (OMNIIDL_INCLUDE_DIRECTORIES)
  SET (_OMNIIDL_INCLUDE_FLAG "")
  FOREACH (DIR ${ARGV})
    SET (_OMNIIDL_INCLUDE_FLAG ${_OMNIIDL_INCLUDE_FLAG}
      -I${DIR} " "
      )
  ENDFOREACH ()
  STRING (REGEX REPLACE " " ";" _OMNIIDL_INCLUDE_FLAG ${_OMNIIDL_INCLUDE_FLAG})
ENDMACRO ()

#.rst:
# .. command:: GENERATE_IDL_CPP (FILENAME DIRECTORY)
#
#   Generate C++ stubs from an idl file.
#   An include directory can also be specified.
#   The filename of the generated file is appended to ``ALL_IDL_CPP_STUBS``.
#
#   In CMake, *source file properties are visible only to targets added in the
#   same directory (CMakeLists.txt)*. As a result, we cannot provide a single
#   macro that takes care of generating the files and ensures a proper build
#   dependency graph.
#
#   .. warning::
#     It is your responsibility to make sure the target dependency tree
#     is correct. For instance with::
#    
#       ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
#       ADD_DEPENDENCIES (my-library generate_idl_cpp)
#
#   For more information:
#   http://www.cmake.org/Wiki/CMake_FAQ#How_can_I_add_a_dependency_to_a_source_file_which_is_generated_in_a_subdirectory.3F
#
#   :param FILENAME:   IDL filename without the extension.
#                      Can be prefixed by a path: _path/_filename
#   :param DIRECTORY:  IDL directory.
#                      The idl file being search for is: ``${DIRECTORY}/${_filename}.idl``
#   :param ENABLE_Wba: Option to trigger generation of code for TypeCode and Any.
#   :param HEADER_SUFFIX: Set option -Wbh of omniidl
#   :param NO_DEFAULT: Do not add default arguments to omniidl (``-Wbkeep_inc_path``)
#   :param ARGUMENTS:  The following words are passed as arguments to omniidl
#
MACRO(GENERATE_IDL_CPP FILENAME DIRECTORY)
  SET(options ENABLE_Wba NO_DEFAULT)
  SET(oneValueArgs HEADER_SUFFIX)
  SET(multiValueArgs ARGUMENTS)
  CMAKE_PARSE_ARGUMENTS(_omni "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  IF(NOT DEFINED _omni_HEADER_SUFFIX)
    SET(_omni_HEADER_SUFFIX ".hh")
  ENDIF()

  GET_FILENAME_COMPONENT (_PATH ${FILENAME} PATH)
  GET_FILENAME_COMPONENT (_NAME ${FILENAME} NAME)
  IF (_PATH STREQUAL "")
    SET(_PATH "./")
  ENDIF (_PATH STREQUAL "")
  FIND_PROGRAM(OMNIIDL omniidl)
  IF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)
    MESSAGE(FATAL_ERROR "cannot find omniidl.")
  ENDIF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)

  SET(IDL_COMPILED_FILES ${FILENAME}SK.cc ${FILENAME}${_omni_HEADER_SUFFIX})
  SET(_omniidl_args -bcxx ${_OMNIIDL_INCLUDE_FLAG} -Wbh=${_omni_HEADER_SUFFIX} ${_omni_ARGUMENTS})
  # This is to keep backward compatibility
  IF(NOT _omni_NO_DEFAULT)
    SET(_omniidl_args ${_omniidl_args} -Wbkeep_inc_path)
  ENDIF()
  IF(_omni_ENABLE_Wba)
    SET(_omniidl_args ${_omniidl_args} -Wba)
    SET(IDL_COMPILED_FILES ${IDL_COMPILED_FILES} ${FILENAME}DynSK.cc)
  ENDIF(_omni_ENABLE_Wba)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${IDL_COMPILED_FILES}
    COMMAND ${OMNIIDL}
    ARGS ${_omniidl_args} -C${_PATH} ${DIRECTORY}/${_NAME}.idl
    MAIN_DEPENDENCY ${DIRECTORY}/${_NAME}.idl
    COMMENT "Generating C++ stubs for ${_NAME}"
    )

  LIST(APPEND ALL_IDL_CPP_STUBS ${IDL_COMPILED_FILES})

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${IDL_COMPILED_FILES}
    )
  SET_PROPERTY(SOURCE ${IDL_COMPILED_FILES}
    APPEND_STRING PROPERTY
    COMPILE_FLAGS "-Wno-conversion -Wno-cast-qual -Wno-unused-variable -Wno-unused-parameter")

  LIST(APPEND LOGGING_WATCHED_VARIABLES OMNIIDL ALL_IDL_CPP_STUBS)
ENDMACRO(GENERATE_IDL_CPP FILENAME DIRECTORY)

#.rst:
# .. command:: GENERATE_IDL_PYTHON (FILENAME DIRECTORY)
#
#   Generate Python stubs from an idl file.
#   An include directory can also be specified.
#   The filename of the generated file is appended to ``ALL_IDL_PYTHON_STUBS``.
#
#   In CMake, *source file properties are visible only to targets added in the
#   same directory (CMakeLists.txt)*. As a result, we cannot provide a single
#   macro that takes care of generating the files and ensures a proper build
#   dependency graph.
#
#   .. warning::
#     It is your responsibility to make sure the target dependency tree
#     is correct. For instance with::
#    
#       ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})
#       ADD_DEPENDENCIES (my-library generate_idl_python)
#
#   For more information:
#   http://www.cmake.org/Wiki/CMake_FAQ#How_can_I_add_a_dependency_to_a_source_file_which_is_generated_in_a_subdirectory.3F
#
#   :param FILENAME: IDL filename without the extension.
#                    Can be prefixed by a path: _path/_filename
#   :param DIRECTORY: IDL directory.
#                     The idl file being search for is: ``${DIRECTORY}/${_filename}.idl``
#   :param ARGUMENTS:  The following words are passed as arguments to omniidl
#   :param ENABLE_DOCSTRING: generate docstrings from doxygen comments (only in Python 3)
#   :param STUBS:    set option -Wbstubs of omniidl.
#
MACRO(GENERATE_IDL_PYTHON FILENAME DIRECTORY)
  SET(options ENABLE_DOCSTRING)
  SET(oneValueArgs STUBS)
  SET(multiValueArgs ARGUMENTS)
  CMAKE_PARSE_ARGUMENTS(_omni "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  GET_FILENAME_COMPONENT (_PATH ${FILENAME} PATH)
  GET_FILENAME_COMPONENT (_NAME ${FILENAME} NAME)
  IF (_PATH STREQUAL "")
    SET(_PATH "./")
  ENDIF (_PATH STREQUAL "")
  FIND_PROGRAM(OMNIIDL omniidl)
  IF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)
    MESSAGE(FATAL_ERROR "cannot find omniidl.")
  ENDIF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)

  IF(_omni_ENABLE_DOCSTRING AND PYTHON_VERSION_MAJOR EQUAL 3)
    SET(_omniidl_args -p${PROJECT_JRL_CMAKE_MODULE_DIR}/hpp/idl -bomniidl_be_python_with_docstring -K)
  ELSE()
    SET(_omniidl_args -bpython)
  ENDIF()
  SET(_omniidl_args ${_omniidl_args} ${_OMNIIDL_INCLUDE_FLAG} -C${_PATH} ${_omni_ARGUMENTS})
  IF(DEFINED _omni_STUBS)
    SET(_omniidl_args ${_omniidl_args} -Wbstubs=${_omni_STUBS})
    STRING(REPLACE "." "/" _omni_STUBS_DIR ${_omni_STUBS})
  ENDIF()
  SET(output_files ${CMAKE_CURRENT_BINARY_DIR}/${_PATH}/${_omni_STUBS_DIR}/${FILENAME}_idl.py)

  ADD_CUSTOM_COMMAND(
    OUTPUT ${output_files}
    COMMAND ${OMNIIDL}
    ARGS ${_omniidl_args}
    ${DIRECTORY}/${_NAME}.idl
    MAIN_DEPENDENCY ${DIRECTORY}/${_NAME}.idl
    COMMENT "Generating Python stubs for ${_NAME}"
    )

  LIST(APPEND ALL_IDL_PYTHON_STUBS ${output_files})

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${output_files}
    )

  LIST(APPEND LOGGING_WATCHED_VARIABLES OMNIIDL ALL_IDL_PYTHON_STUBS)
ENDMACRO(GENERATE_IDL_PYTHON FILENAME DIRECTORY)

# GENERATE_IDL_FILE FILENAME DIRECTORY
# ------------------------------------
#
# Legacy macro, now replaced by GENERATE_IDL_CPP.
MACRO(GENERATE_IDL_FILE FILENAME DIRECTORY)
  MESSAGE(FATAL_ERROR
    "GENERATE_IDL_FILE has been removed. Please use GENERATE_IDL_CPP instead.")
ENDMACRO(GENERATE_IDL_FILE FILENAME DIRECTORY)
