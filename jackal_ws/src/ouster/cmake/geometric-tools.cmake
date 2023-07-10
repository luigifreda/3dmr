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

# SEARCH_FOR_GEOMETRIC_TOOLS
# -----------------
#
# The geometric-tools (aka WildMagic5) does not provide a pkg-config
# file. This macro defines a CMake variable that must be filled to
# point to the geometric-tools install prefix.
#
MACRO(SEARCH_FOR_GEOMETRIC_TOOLS)
  MESSAGE(STATUS "geometric-tools is required.")
  SET(GEOMETRIC_TOOLS_INSTALL_PREFIX "" CACHE PATH "geometric-tools installation prefix")
  SET(LIB_GEOMETRIC_TOOLS_CORE LIB_GEOMETRIC_TOOLS_CORE-NOTFOUND)
  SET(LIB_GEOMETRIC_TOOLS_MATH LIB_GEOMETRIC_TOOLS_MATH-NOTFOUND)
  MESSAGE(STATUS "checking for module geometric-tools")
  FIND_LIBRARY(LIB_GEOMETRIC_TOOLS_CORE
    libWm5Core.so
    PATH
    ${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib)
  IF (NOT LIB_GEOMETRIC_TOOLS_CORE)
    MESSAGE(FATAL_ERROR
      "Failed to find geometric-tools Core library, check that geometric-tools is installed and set the GEOMETRIC_TOOLS_INSTALL_PREFIX CMake variable.")
  ENDIF()
  FIND_LIBRARY(LIB_GEOMETRIC_TOOLS_MATH
    libWm5Mathematics.so
    PATH
    ${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib)
  IF (NOT LIB_GEOMETRIC_TOOLS_MATH)
    MESSAGE(FATAL_ERROR
      "Failed to find geometric-tools Mathematics library, check that geometric-tools is installed and set the GEOMETRIC_TOOLS_INSTALL_PREFIX CMake variable.")
  ENDIF()
  SET(GEOMETRIC_TOOLS_H GEOMETRIC_TOOLS-NOTFOUND)
  FIND_PATH (GEOMETRIC_TOOLS_H
    Wm5DistSegment3Segment3.h
    "${GEOMETRIC_TOOLS_INSTALL_PREFIX}/include/geometric-tools")
  IF (NOT GEOMETRIC_TOOLS_H)
    MESSAGE(FATAL_ERROR
      "Failed to find geometric-tools/Wm5DistSegment3Segment3.h, check that geometric-tools is installed.")
  ENDIF()

  MESSAGE(STATUS "  found geometric-tools")

  SET(GEOMETRIC_TOOLS_INCLUDEDIR "${GEOMETRIC_TOOLS_INSTALL_PREFIX}/include")
  SET(GEOMETRIC_TOOLS_LIBRARYDIR "${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib")
  SET(GEOMETRIC_TOOLS_LIBRARIES
    ${LIB_GEOMETRIC_TOOLS_MATH} ${LIB_GEOMETRIC_TOOLS_CORE})

  INCLUDE_DIRECTORIES(SYSTEM ${GEOMETRIC_TOOLS_INCLUDEDIR})
  LINK_DIRECTORIES(${GEOMETRIC_TOOLS_LIBRARYDIR})

  PKG_CONFIG_APPEND_CFLAGS("-isystem ${GEOMETRIC_TOOLS_INCLUDEDIR}")
  PKG_CONFIG_APPEND_LIBRARY_DIR("${GEOMETRIC_TOOLS_LIBRARYDIR}")

  MESSAGE(STATUS "Module geometric-tools has been detected with success.")
ENDMACRO(SEARCH_FOR_GEOMETRIC_TOOLS)
