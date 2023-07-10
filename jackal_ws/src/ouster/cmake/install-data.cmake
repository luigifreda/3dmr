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

# _INSTALL_PROJECT_DATA
# ---------------------
#
# Build the installation rules to install data-files.
#
MACRO(_INSTALL_PROJECT_DATA)
  IF(DEFINED PROJECT_DATA)
    INSTALL(FILES ${PROJECT_DATA}
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      )
  ENDIF(DEFINED PROJECT_DATA)
ENDMACRO(_INSTALL_PROJECT_DATA)
