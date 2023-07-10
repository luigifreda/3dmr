# Copyright (C) 2017-2020 LAAS-CNRS, JRL AIST-CNRS.
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

include(CheckCCompilerFlag)
# Introduced in 3.18.0 but VERSION_GREAT_OR_EQUAL is not available in CMake 3.1
if(${CMAKE_VERSION} VERSION_GREATER 3.17.6)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0057 NEW) # if IN_LIST
  include(CheckLinkerFlag)
  cmake_policy(POP)
endif()

# _CHECK_VERSION_SCRIPT_SUPPORT
# -----------------------------
#
# Internal macro to check if version scripts are supported by the current
# linker.
macro(_CHECK_VERSION_SCRIPT_SUPPORT)
  set(VERSION_SCRIPT "${PROJECT_JRL_CMAKE_MODULE_DIR}/version-script-test.lds")
  if(COMMAND check_linker_flag)
    check_linker_flag("C" "-Wl,--version-script=${VERSION_SCRIPT}" HAS_VERSION_SCRIPT_SUPPORT)
  else()
    check_c_compiler_flag("-Wl,--version-script=${VERSION_SCRIPT}" HAS_VERSION_SCRIPT_SUPPORT)
  endif()
  set(_HAS_VERSION_SCRIPT_SUPPORT ${HAS_VERSION_SCRIPT_SUPPORT} CACHE INTERNAL "Linker supports version scripts")
endmacro(_CHECK_VERSION_SCRIPT_SUPPORT)

#.rst:
# .. command:: ADD_VERSION_SCRIPT(TARGET VERSION_SCRIPT)
#
#   This macro adds a version script to a given target and a link-time
#   dependency between the target and the version script.
#
#   See https://www.gnu.org/software/gnulib/manual/html_node/LD-Version-Scripts.html
#
#   It has no effect on platforms that do not support version script.
#
#   :param TARGET:         Name of the target, the macro does nothing if TARGET is not a
#                          cmake target.
#   :param VERSION_SCRIPT: Version script to add to the library.
#
macro(ADD_VERSION_SCRIPT TARGET VERSION_SCRIPT)
  if(NOT DEFINED _HAS_VERSION_SCRIPT_SUPPORT)
    _CHECK_VERSION_SCRIPT_SUPPORT()
  endif()
  if(_HAS_VERSION_SCRIPT_SUPPORT)
    if(TARGET ${TARGET})
      set_property(TARGET ${TARGET} APPEND_STRING PROPERTY
                   LINK_FLAGS " -Wl,--version-script=${VERSION_SCRIPT}")
      set_target_properties(${TARGET} PROPERTIES LINK_DEPENDS ${VERSION_SCRIPT})
    endif()
  endif()
endmacro(ADD_VERSION_SCRIPT)
