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

##########
# README #
##########
#
# This CMake file implements various substitutes to standard CMake macros and
# functions in order to simplify them or make them more robust.
#
# Indeed, many standard macros/functions interface have changed depending
# on your CMake version triggering many problems when working with too recent
# or too old versions of CMake.
#
# These macros should be used instead of the CMake one to enhance package
# robustness.

# CONFIG_FILES
# ---------------
#
# This wraps CONFIGURE_FILES to provide a cleaner, shorter syntax.
#
FUNCTION(CONFIG_FILES)
  FOREACH(I RANGE 0 ${ARGC})
    SET(FILE ${ARGV${I}})
    IF(FILE)
      CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/${FILE}.in
	${CMAKE_CURRENT_BINARY_DIR}/${FILE}
	@ONLY
	)
    ENDIF(FILE)
ENDFOREACH(I RANGE 0 ${ARGC})
ENDFUNCTION(CONFIG_FILES)

# CONFIG_FILES_CMAKE
# ------------------
#
# Same as CONFIG_FILES but with CMake-style template files.
#
# Please, prefer the use of CONFIG_FILES to this function as
# it is safer.
#
FUNCTION(CONFIG_FILES_CMAKE)
  FOREACH(I RANGE 0 ${ARGC})
    SET(FILE ${ARGV${I}})
    IF(FILE)
      CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/${FILE}.cmake
	${CMAKE_CURRENT_BINARY_DIR}/${FILE}
	)
    ENDIF(FILE)
ENDFOREACH(I RANGE 0 ${ARGC})
ENDFUNCTION(CONFIG_FILES_CMAKE)


# NORMALIZE_PATH
# ------------------
#
# Convert the windows style path into unix style path.
#
# On windows, the folder separator is \, wihch can lead to some issues 
# because of the appearance of special characters like \p, \n ...
#
FUNCTION(NORMALIZE_PATH mypath)
  IF(WIN32)
    STRING(REPLACE "\\" "/" ${mypath} "${${mypath}}")
	SET(${mypath} ${${mypath}} PARENT_SCOPE)
  ENDIF(WIN32)
ENDFUNCTION(NORMALIZE_PATH)

