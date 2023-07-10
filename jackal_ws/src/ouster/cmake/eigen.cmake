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
# .. command:: SEARCH_FOR_EIGEN
#
#   This macro gets eigen include path from pkg-config file, and adds it include directories.
#   If there is no pkg-config for Eigen, we fall back to a manual search.
#
#   If no version requirement argument is passed to the macro, it looks for the
#   variable Eigen_REQUIRED. If this variable is not defined before calling
#   the method SEARCH_FOR_EIGEN, the minimum version requirement is 3.0.0 by default.
MACRO(SEARCH_FOR_EIGEN)
  # ref https://github.com/jrl-umi3218/jrl-cmakemodules/issues/319
  MESSAGE(AUTHOR_WARNING "SEARCH_FOR_EIGEN is deprecated and will be removed in the future")
  SET(_Eigen_FOUND 0)
  IF(${ARGC} GREATER 0)
    SET(Eigen_REQUIRED ${ARGV0})
  ELSEIF(NOT Eigen_REQUIRED)
    SET(Eigen_REQUIRED "eigen3 >= 3.0.0")
  ENDIF()

  # looking for .pc
  PKG_CHECK_MODULES(_Eigen ${Eigen_REQUIRED})

  IF(${_Eigen_FOUND})
    SET(${PROJECT_NAME}_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} ${_Eigen_CFLAGS_OTHER}")
    SET(${PROJECT_NAME}_LINK_FLAGS "${${PROJECT_NAME}_LINK_FLAGS} ${_Eigen_LDFLAGS_OTHER}")

    INCLUDE_DIRECTORIES(SYSTEM ${_Eigen_INCLUDE_DIRS} )
    _ADD_TO_LIST(_PKG_CONFIG_REQUIRES "${Eigen_REQUIRED}" ",")
  ELSE()
    # fallback: search for the signature_of_eigen3_matrix_library file
    FIND_PATH(Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
      PATHS
      ${CMAKE_INSTALL_PREFIX}/include
      PATH_SUFFIXES eigen3 eigen
    )
    INCLUDE_DIRECTORIES(SYSTEM ${Eigen_INCLUDE_DIR})
    PKG_CONFIG_APPEND_CFLAGS(-I"${Eigen_INCLUDE_DIR}")
  ENDIF()
ENDMACRO(SEARCH_FOR_EIGEN)
