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

# _SETUP_PROJECT_UNINSTALL
# ------------------------
#
# Add custom rule to uninstall the package.
#
MACRO(_SETUP_PROJECT_UNINSTALL)
  # FIXME: it is utterly stupid to rely on the install manifest.
  # Can't we just remember what we install ?!
  CONFIGURE_FILE(
    "${CMAKE_CURRENT_LIST_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
    IMMEDIATE
    @ONLY
    )

  ADD_CUSTOM_TARGET(
    uninstall
    "${CMAKE_COMMAND}" -P
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
    )

  CONFIGURE_FILE(
    "${CMAKE_CURRENT_LIST_DIR}/cmake_reinstall.cmake.in"
    "${PROJECT_BINARY_DIR}/cmake/cmake_reinstall.cmake.configured"
    )
  IF(DEFINED CMAKE_BUILD_TYPE)
    FILE(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/cmake/${CMAKE_BUILD_TYPE}")
  ELSE(DEFINED CMAKE_BUILD_TYPE)
    FOREACH(CFG ${CMAKE_CONFIGURATION_TYPES})
      FILE(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/cmake/${CFG}")
    ENDFOREACH()
  ENDIF(DEFINED CMAKE_BUILD_TYPE)
  FILE(GENERATE
    OUTPUT "${PROJECT_BINARY_DIR}/cmake/$<CONFIGURATION>/cmake_reinstall.cmake"
    INPUT "${PROJECT_BINARY_DIR}/cmake/cmake_reinstall.cmake.configured"
    )
  ADD_CUSTOM_TARGET(
    reinstall
    "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/cmake/$<CONFIGURATION>/cmake_reinstall.cmake"
    )
ENDMACRO(_SETUP_PROJECT_UNINSTALL)
