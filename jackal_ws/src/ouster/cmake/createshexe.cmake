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

# CREATE_SH_EXE
# -------------------
#
# Add a deb target to generate a Debian package using
# git-buildpackage (Linux specific).
#
MACRO(CREATE_SH_EXE EXEC_NAME)
  IF(UNIX)  
    SET(sh_exe_filename "${EXEC_NAME}.sh")
    FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${sh_exe_filename} "#!/bin/sh\nLD_LIBRARY_PATH=$LD_LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/lib/plugin;export LD_LIBRARY_PATH\n./${EXEC_NAME}")
    INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/${sh_exe_filename} 
      DESTINATION bin
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
      GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ)
  ENDIF(UNIX)
ENDMACRO(CREATE_SH_EXE)
