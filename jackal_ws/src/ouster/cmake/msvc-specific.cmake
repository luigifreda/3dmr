# Copyright (C) 2016-2021 JRL AIST-CNRS.
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

#
# This file contains a collection of macros specific to Visual Studio and the
# MSVC compilers.
#

# GET_MSVC_VERSION
# ----------------
#
# Deprecated
#
# This macro return a string corresponding to the version of the MSVC compiler
# used.
# The value is returned in the variable MSVC_VERSION. It is an empty string if
#  - the compiler used is not MSVC,
#  - it is older than MSVC 6.0
#  - or newer than MSVC 14.0
#
MACRO(GET_MSVC_VERSION)
  message(DEPRECATION "This macro is deprecated. Use GET_MSVC_TOOLS_VERSION instead.")
  if(MSVC60)
    set(MSVC_VERSION "6.0")
  elseif(MSVC70)
    set(MSVC_VERSION "7.0")
  elseif(MSVC71)
    set(MSVC_VERSION "7.1")
  elseif(MSVC80)
    set(MSVC_VERSION "8.0")
  elseif(MSVC90)
    set(MSVC_VERSION "9.0")
  elseif(MSVC10)
    set(MSVC_VERSION "10.0")
  elseif(MSVC11)
    set(MSVC_VERSION "11.0")
  elseif(MSVC12)
    set(MSVC_VERSION "12.0")
  elseif(MSVC14)
    set(MSVC_VERSION "14.0")
  else()
    if(MSVC)
      set(MSVC_VERSION "14.0")
      message(AUTHOR_WARNING "MSVC version not found. Set MSVC_VERSION to 14.0 by default. Please update the GET_MSVC_VERSION macro.")
    else()
      set(MSVC_VERSION "")
      message(AUTHOR_WARNING "MSVC version not found. You are not using a MSVC generator.")
    endif()
  endif()
ENDMACRO(GET_MSVC_VERSION)


# GET_MSVC_TOOLS_VERSION
# ----------------------
#
# This macro return a string corresponding to the version of the MSVC compiler
# used.
# The value is returned in the variable MSVC_TOOLS_VERSION. It is an empty
# string if the compiler used is not MSVC.
#
MACRO(GET_MSVC_TOOLS_VERSION)
  string(REGEX MATCH "\\." CONTAINS_DOT ${MSVC_VERSION})
  if(CONTAINS_DOT)
    message(AUTHOR_WARNING "MSVC_VERSION has been overwritten with a tools version number (likely by a call to deprecated GET_MSVC_VERSION. Using this number.")
    set(MSVC_TOOLS_VERSION ${MSVC_VERSION})
  else()
    if(MSVC_VERSION EQUAL 1200)
      set(MSVC_TOOLS_VERSION "6.0")
    elseif(MSVC_VERSION EQUAL 1300)
      set(MSVC_TOOLS_VERSION "7.0")
    elseif(MSVC_VERSION EQUAL 1310)
      set(MSVC_TOOLS_VERSION "7.1")
    elseif(MSVC_VERSION EQUAL 1400)
      set(MSVC_TOOLS_VERSION "8.0")
    elseif(MSVC_VERSION EQUAL 1500)
      set(MSVC_TOOLS_VERSION "9.0")
    elseif(MSVC_VERSION EQUAL 1600)
      set(MSVC_TOOLS_VERSION "10.0")
    elseif(MSVC_VERSION EQUAL 1700)
      set(MSVC_TOOLS_VERSION "11.0")
    elseif(MSVC_VERSION EQUAL 1800)
      set(MSVC_TOOLS_VERSION "12.0")
    elseif(MSVC_VERSION EQUAL 1900)
      set(MSVC_TOOLS_VERSION "14.0")
    elseif((MSVC_VERSION GREATER_EQUAL 1910) AND (MSVC_VERSION LESS 1920))
      set(MSVC_TOOLS_VERSION "15.0")
    elseif((MSVC_VERSION GREATER_EQUAL 1920) AND (MSVC_VERSION LESS 1930))
      set(MSVC_TOOLS_VERSION "16.0")
    else()
      if(MSVC)
        set(MSVC_TOOLS_VERSION "16.0")
        message(AUTHOR_WARNING "MSVC version not found. Assuming newer version and set MSVC_TOOLS_VERSION to 16.0 by default. Please update the GET_MSVC_TOOLS_VERSION macro.")
      else()
        set(MSVC_TOOLS_VERSION "")
        message(AUTHOR_WARNING "MSVC version not found. You are not using a MSVC generator.")
      endif()
    endif()
  endif()
  unset(CONTAINS_DOT)
ENDMACRO()


# REQUEST_MINIMUM_MSVC_VERSION(VER)
# ---------------------------------
#
# Return a fatal error if the current MSVC version is strictly lower than VER
# or when no MSVC compiler or an unknown version of it is used.
#
# A reason for the version to be unknown is that it is newer than the versions
# recognize by the above macro GET_MSVC_TOOLS_VERSION. In this case, please update 
# the macro and its documentation.
#
MACRO(REQUIRE_MINIMUM_MSVC_VERSION VERSION)
  GET_MSVC_TOOLS_VERSION()
  if (${MSVC_TOOLS_VERSION})
    if (NOT ${MSVC_TOOLS_VERSION} VERSION_GREATER ${VERSION})
	  message(FATAL_ERROR "Minimum MSVC version required is " ${VERSION} " but version " ${MSVC_TOOLS_VERSION} " was found.")
    endif()
  else()
    message(FATAL_ERROR "You are requiring a minimum version for MSVC but you do not use a MSVC generator.")
  endif(${MSVC_TOOLS_VERSION})
ENDMACRO(REQUIRE_MINIMUM_MSVC_VERSION)

if(${CMAKE_VERSION} VERSION_LESS "3.5.0") 
    include(CMakeParseArguments)
endif()


# GENERATE_MSVC_DOT_USER_FILE(<name> [<additional_path>])
# GENERATE_MSVC_DOT_USER_FILE(NAME <name> [COMMAND <command>] 
#                             [COMMAND_ARGS <args>] 
#                             [WORKING_DIRECTORY <dir>]
#                             [ADDITIONAL_PATH <additional_path>])
# ----------------------------------------------------------------
#
# Generate the file <name>.vcxproj.user alongside the project file
# <name>.vcxproj that sets up the options for a local debugging session.
# This is intended to be used for the test projects, so that they can be 
# launched from within the Visual Studio IDE by pressing F5.
# The project will be executed with the command <command> <args> if specified
# with <dir> as the working directory.
# The %PATH% environment variable is automatically extended to include the 
# location of the libraries generated by the solution, i.e. it sets the field
# Configuration Properties > Debuging > Environment to 
# PATH=$(SolutionDir)\src\$(Configuration);additional_path;%PATH%
# <additional_path> is an optional semicolon-separated list of paths.
#
MACRO(GENERATE_MSVC_DOT_USER_FILE)
  if(MSVC)
    REQUIRE_MINIMUM_MSVC_VERSION("10.0")
    
    set(oneValueArgs NAME COMMAND COMMAND_ARGS WORKING_DIRECTORY ADDITIONAL_PATH)
    cmake_parse_arguments(GMDUT "" "${oneValueArgs}" "" ${ARGN} )
    
    if((NOT GMDUT_NAME) AND (NOT GMDUT_COMMAND) AND (NOT GMDUT_COMMAND_ARGS) 
        AND (NOT GMDUT_WORK_DIR) AND (NOT GMDUT_ADDITIONAL_PATH))
      #legacy version
      set(GMDUT_NAME ${ARGV0})
      if(${ARGC} GREATER 1)
        set(MSVC_DOT_USER_ADDITIONAL_PATH_DOT_USER ";${ARGV1}")
      endif()
    endif()
    
    if("${CMAKE_MSVCIDE_RUN_PATH}" STREQUAL "")
      set(CMAKE_MSVCIDE_RUN_PATH "$(SolutionDir)\src\$(Configuration)")
    endif()
    
    if(NOT GMDUT_NAME)
      message(FATAL_ERROR "NAME argument is required.")
    endif()
    if(NOT GMDUT_COMMAND)
      set(GMDUT_COMMAND "$(TargetPath)")
    endif()
    if(NOT GMDUT_WORKING_DIRECTORY)
      set(GMDUT_WORKING_DIRECTORY "$(ProjectDir)")
    endif()
    if(GMDUT_ADDITIONAL_PATH)
      set(MSVC_DOT_USER_ADDITIONAL_PATH_DOT_USER ";${GMDUT_ADDITIONAL_PATH}")
    endif()
    
    GET_MSVC_TOOLS_VERSION()
    set(DOT_USER_TEMPLATE_PATH ${PROJECT_JRL_CMAKE_MODULE_DIR}/msvc.vcxproj.user.in)
    set(DOT_USER_FILE_PATH ${CMAKE_CURRENT_BINARY_DIR}/${GMDUT_NAME}.vcxproj.user)
    configure_file(${DOT_USER_TEMPLATE_PATH} ${DOT_USER_FILE_PATH})
    
    unset(MSVC_DOT_USER_ADDITIONAL_PATH_DOT_USER)
  endif(MSVC)
ENDMACRO(GENERATE_MSVC_DOT_USER_FILE)
