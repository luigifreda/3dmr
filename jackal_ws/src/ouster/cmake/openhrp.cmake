# Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS, LIRMM-CNRS
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


# Search if openhrp3.1 is installed.
# Set the value GRX_PREFIX if found
MACRO(SEARCH_GRX)
  # Define additional options.
  FIND_PATH(GRX_PREFIX
    # Make sure bin/DynamicsSimulator exists (to validate that it is
    # _really_ the prefix of an OpenHRP setup).
    include/rtm/idl/SDOPackage.hh
    HINTS /opt/grx/
    DOC "GRX software prefix (i.e. '/opt/grxX.Y')"
    NO_DEFAULT_PATH)
ENDMACRO(SEARCH_GRX)


# Search if openhrp3.0(.7) is installed.
# Set the value GRX_PREFIX if found
MACRO(SEARCH_GRX3)
  FIND_PATH(GRX_PREFIX
    # Make sure bin/DynamicsSimulator exists (to validate that it is
    # _really_ the prefix of an OpenHRP setup).
    OpenHRP/DynamicsSimulator/server/DynamicsSimulator
    HINTS /opt/grx3.0
    DOC "GRX software prefix (i.e. '/opt/grxX.Y')"
    NO_DEFAULT_PATH)
ENDMACRO(SEARCH_GRX3)


# Check the robots installed in openhrp
# Args: the robot researched (and handled by the package).
# Return: the list GRX_ROBOTS
# Example: SEARCH_GRX_ROBOTS("robot1;robot2")
MACRO(SEARCH_GRX_ROBOTS  HANDLED_ROBOTS)
  FOREACH(robot ${HANDLED_ROBOTS})
    IF(!GRX_PREFIX)
      MESSAGE(ERROR "Dit not find GRX_PREFIX")
    ELSE(!GRX_PREFIX)
      #List of know robots.
      IF(IS_DIRECTORY ${GRX_PREFIX}/${robot})
        LIST(APPEND GRX_ROBOTS ${robot})
      ENDIF()
    ENDIF(!GRX_PREFIX)
  ENDFOREACH(robot)

  IF(NOT GRX_ROBOTS)
    MESSAGE(FATAL_ERROR
      "None of the following robots (${HANDLED_ROBOTS}) were found in ${GRX_PREFIX}."
    )
  ELSE()
    MESSAGE("The following robots were found: ${GRX_ROBOTS}")
  ENDIF()
  LIST(APPEND LOGGING_WATCHED_VARIABLES GRX_ROBOTS)
ENDMACRO(SEARCH_GRX_ROBOTS)

