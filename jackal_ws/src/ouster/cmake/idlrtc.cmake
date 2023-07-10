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

# GENERATE_IDL_FILE FILENAME DIRECTORY
# ------------------------------------
#
# Generate stubs from an idl file.
# An include directory can also be specified.
#
# FILENAME : IDL filename without the extension
# DIRECTORY : IDL directory
# LIST_INCLUDE_DIRECTORIES (optional) : List of include directories
#
MACRO(GENERATE_IDLRTC_FILE FILENAME DIRECTORY)
  FIND_PROGRAM(OMNIIDL omniidl)

  # Test existence of omniidl
  IF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)
    MESSAGE(FATAL_ERROR "cannot find omniidl.")
  ENDIF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)

  # Create the flag to include directories
  SET(OMNIIDL_INC_DIR "")
  
  # Check if there is an optional value
  MESSAGE(STATUS "ARGC: "${ARGC} )
  IF(${ARGC} EQUAL 3)
    # If there is, the directory to include are added.
    SET(LIST_INCLUDE_DIRECTORIES ${ARGV2})

    MESSAGE(STATUS "ARGV2: "${ARGV2} )
    FOREACH(INCDIR ${LIST_INCLUDE_DIRECTORIES})
      # The format for the first one is special
      # to avoid a \ to be introduced.
      IF(OMNIIDL_INC_DIR STREQUAL "")
	SET(OMNIIDL_INC_DIR "-I${INCDIR}")
      ELSE(OMNIIDL_INC_DIR STREQUAL "")
	SET(OMNIIDL_INC_DIR ${OMNIIDL_INC_DIR} "-I${INCDIR}")
      ENDIF(OMNIIDL_INC_DIR STREQUAL "")
    ENDFOREACH(INCDIR ${LIST_INCLUDE_DIRECTORIES})

  ENDIF(${ARGC} EQUAL 3)

  SET(IDL_FLAGS "-Wbuse_quotes" "-Wbh=.hh" "-Wbs=SK.cc" "-Wba" "-Wbd=DynSK.cc")
  MESSAGE(STATUS "OMNIIDL_INC_DIR:" ${OMNIIDL_INC_DIR})
  ADD_CUSTOM_COMMAND(
    OUTPUT ${FILENAME}SK.cc ${FILENAME}DynSK.cc ${FILENAME}.hh
    COMMAND ${OMNIIDL}
    ARGS -bcxx ${IDL_FLAGS} ${OMNIIDL_INC_DIR} ${DIRECTORY}/${FILENAME}.idl 
    MAIN_DEPENDENCY ${DIRECTORY}/${FILENAME}.idl
    )
  SET(ALL_IDL_STUBS ${FILENAME}SK.cc ${FILENAME}DynSK.cc ${FILENAME}.hh ${ALL_IDL_STUBS})

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${FILENAME}SK.cc ${FILENAME}DynSK.cc ${FILENAME}.hh
    )

  LIST(APPEND LOGGING_WATCHED_VARIABLES OMNIIDL ALL_IDL_STUBS)
ENDMACRO(GENERATE_IDLRTC_FILE FILENAME DIRECTORY)
