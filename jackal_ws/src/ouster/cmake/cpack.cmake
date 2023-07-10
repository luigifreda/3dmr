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

# ADD_CMAKE_DEPENDENCY
# --------------------
# 
# Warning: use on this macro if NECESSARY.
# 
# This macro adds CPack dependencies support to the package.
# It should be avoided unless Ubuntu 8.04 packages have to be built.
#
# Please, prefer the use of git-archive for tarball generation
# and debhelper for Debian package generation.
#
MACRO(ADD_CMAKE_DEPENDENCY PKG_CONFIG_STRING)
  MESSAGE(STATUS "PKG_CONFIG_STRING: ${PKG_CONFIG_STRING}")
  STRING(REGEX MATCH "[^<>= ]+" LIBRARY_NAME "${PKG_CONFIG_STRING}")
  # Carefull the space in front of the matching string is important to avoid
  # confusion with package name.
  STRING(REGEX MATCHALL " [0-9]+.[0-9]+(.[a-z0-9-])*" VERSION "${PKG_CONFIG_STRING}")
  _ADD_TO_LIST(CPACK_INTERNAL_CONFIG_REQUIRES "${LIBRARY_NAME}(>=${VERSION})" ",")
ENDMACRO(ADD_CMAKE_DEPENDENCY)

# SETUP_PROJECT_CPACK
# -------------------
#
# Warning: use only this macro if NECESSARY.
#
# This macro adds CPack support to the package.
# It should be avoided unless Ubuntu 8.04 packages have to be built.
#
# Please, prefer the use of git-archive for tarball generation
# and debhelper for Debian package generation.
#
MACRO(SETUP_PROJECT_CPACK)
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PROJECT_DESCRIPTION})
  SET(CPACK_PACKAGE_VENDOR "JRL CNRS/AIST")
  SET(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

  SET(CPACK_PACKAGE_DESCRIPTION_FILE ${PROJECT_SOURCE_DIR}/README.md)
  SET(CPACK_DEBIAN_PACKAGE_MAINTAINER
    "Olivier Stasse (olivier.stasse@aist.go.jp)")

  # The following components are regex's to match anywhere (unless anchored)
  # in absolute path + filename to find files or directories to be excluded
  # from source tarball.
  SET(CPACK_SOURCE_IGNORE_FILES
    "~$"
    "^${PROJECT_SOURCE_DIR}/build/"
    "^${PROJECT_SOURCE_DIR}/.git/"
    )

  SET(
    CPACK_SOURCE_PACKAGE_FILE_NAME
    "${PROJECT_NAME}-src-${PROJECT_VERSION}"
    CACHE INTERNAL "tarball basename"
    )

  SET(CPACK_PACKAGE_NAME ${PROJECT_NAME})
  SET(CPACK_BINARY_DEB ON)
  SET(CPACK_GENERATOR TGZ)
  SET(CPACK_GENERATOR DEB)
  SET(CPACK_PACKAGING_INSTALL_PREFIX "/opt/openrobots")

  # Set dependencies
  SET(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_INTERNAL_CONFIG_REQUIRES}")

  # CPack SHOULD be called after setting all the variables.
  # SETUP_PROJECT_CPACK is supposed to be called only once for a project.
  INCLUDE(CPack)

ENDMACRO(SETUP_PROJECT_CPACK)
