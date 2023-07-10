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

# Try to find libcdd
# in standard prefixes and in ${CDD_PREFIX}
# Once done this will define
#  CDD_FOUND - System has CDD
#  CDD_INCLUDE_DIRS - The CDD include directories
#  CDD_LIBRARIES - The libraries needed to use CDD
#  CDD_DEFINITIONS - Compiler switches required for using CDD

FIND_PATH(CDD_INCLUDE_DIR
  NAMES cdd.h cddmp.h
  PATHS ${CDD_PREFIX}
  PATH_SUFFIXES include/cdd include/cddlib
  )
FIND_LIBRARY(CDD_LIBRARY
  NAMES libcdd.so
  PATHS ${CDD_PREFIX}
  )

SET(CDD_LIBRARIES ${CDD_LIBRARY})
SET(CDD_INCLUDE_DIRS ${CDD_INCLUDE_DIR})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CDD DEFAULT_MSG CDD_LIBRARY CDD_INCLUDE_DIR)
mark_as_advanced(CDD_INCLUDE_DIR CDD_LIBRARY)
