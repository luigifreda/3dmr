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

# SEARCH_FOR_LAPACK
# -----------------
#
# Search for LAPACK in a portable way.
#
# This macro deals with Visual Studio Fortran incompatibilities
# and add detected flags to the pkg-config file automatically.
#
MACRO(SEARCH_FOR_LAPACK)
  IF(NOT "${CMAKE_GENERATOR}" MATCHES "Visual Studio.*")
    ENABLE_LANGUAGE(Fortran)
  ENDIF(NOT "${CMAKE_GENERATOR}" MATCHES "Visual Studio.*")

  FIND_PACKAGE(LAPACK REQUIRED)

  IF(NOT LAPACK_FOUND)
    MESSAGE(FATAL_ERROR "Failed to detect LAPACK.")
  ENDIF(NOT LAPACK_FOUND)

  IF(WIN32)
    # Enabling Fortran on Win32 causes the definition of variables
    #  that change the name of the library built, add the prefix 'lib'
    # These commands are Counter CMake mesures:
    SET(CMAKE_STATIC_LIBRARY_PREFIX "")
    SET(CMAKE_SHARED_LIBRARY_PREFIX "")
    SET(CMAKE_SHARED_MODULE_PREFIX "")
    SET(CMAKE_LINK_LIBRARY_SUFFIX ".lib")
  ENDIF(WIN32)

  PKG_CONFIG_APPEND_LIBS_RAW("${LAPACK_LINKER_FLAGS};${LAPACK_LIBRARIES}")

  # Watch variables.
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    LAPACK_FOUND
    LAPACK_LINKER_FLAGS
    LAPACK_LIBRARIES
    LAPACK95_LIBRARIES
    LAPACK95_FOUND
    BLA_STATIC
    BLA_VENDOR
    BLA_F95
    )
ENDMACRO(SEARCH_FOR_LAPACK)
