#
#   Copyright 2019 CNRS
#
#   Author: Guilhem Saurel
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

# Try to find glpk
# in standard prefixes and in ${glpk_PREFIX}
# Once done this will define
#  glpk_FOUND - System has glpk
#  glpk_INCLUDE_DIRS - The glpk include directories
#  glpk_LIBRARIES - The libraries needed to use glpk
#  glpk_DEFINITIONS - Compiler switches required for using glpk

FIND_PATH(glpk_INCLUDE_DIR
  NAMES glpk.h
  PATHS ${glpk_PREFIX}
  )
FIND_LIBRARY(glpk_LIBRARY
  NAMES libglpk.so
  PATHS ${glpk_PREFIX}
  PATH_SUFFIXES include/glpk
  )

SET(glpk_LIBRARIES ${glpk_LIBRARY})
SET(glpk_INCLUDE_DIRS ${glpk_INCLUDE_DIR})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(glpk DEFAULT_MSG glpk_LIBRARY glpk_INCLUDE_DIR)
MARK_AS_ADVANCED(glpk_INCLUDE_DIR glpk_LIBRARY)
