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

# Shared library related constants
# (used for pkg-config file generation).
# FIXME: can't we get these information from CMake directly?
SET(LIBDIR_KW "-L")
SET(LIBINCL_KW "-l")
SET(LIBINCL_ABSKW "")
SET(LIB_EXT "")
IF(WIN32)
  SET(LIB_EXT ".lib")
ELSEIF(UNIX)
  # Using -l:/some/absolute/path.so was an "undocumented ld feature, in
  # actual fact a ld bug, that has since been fixed".
  # This was apparently used (e.g. in ROS) because of pkg-config problems that
  # have since been fixed.
  # See: https://github.com/ros/catkin/issues/694#issuecomment-88323282
  # Note: ld version on Linux can be 2.25.1 or 2.24
  IF (NOT CMAKE_LINKER)
    INCLUDE(CMakeFindBinUtils)
  ENDIF()

  EXECUTE_PROCESS(COMMAND ${CMAKE_LINKER} -v OUTPUT_VARIABLE LD_VERSION_STR ERROR_VARIABLE LD_VERSION_STR)
  STRING(REGEX MATCH "([0-9]+\\.[0-9]+(\\.[0-9]+)?)" LD_VERSION ${LD_VERSION_STR})
  IF(${LD_VERSION} VERSION_LESS "2.24.90")
    SET(LIBINCL_ABSKW "-l:")
  ELSE()
    SET(LIBINCL_ABSKW "")
  ENDIF()
ENDIF(WIN32)
