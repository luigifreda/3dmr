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

# _SETUP_DEBIAN
# -------------------
#
# Prepare needed file for debian if the target is a shared library.
#
MACRO(_SETUP_DEBIAN)
  IF (BUILDING_DEBIAN_PACKAGE)
    MESSAGE(STATUS "setup debian. Trying to get the type of ${PROJECT_NAME}")
    GET_TARGET_PROPERTY(${PROJECT_NAME}_IS_SHARED_LIBRARY ${PROJECT_NAME} TYPE)
    MESSAGE(STATUS "${PROJECT_NAME}_IS_SHARED_LIBRARY:" ${${PROJECT_NAME}_IS_SHARED_LIBRARY})

    IF(UNIX)
      IF (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/debian)
	IF(${PROJECT_NAME}_IS_SHARED_LIBRARY STREQUAL "SHARED_LIBRARY")
	  # Create the install file to be inside ld.so.conf.d
          MESSAGE(STATUS "PROJECT_SOURCE_DIR:" ${PROJECT_SOURCE_DIR})
 	  EXECUTE_PROCESS(
	    COMMAND ${GIT} describe --abbrev=0 --match=v* HEAD
	    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	    RESULT_VARIABLE LGIT_DESCRIBE_RESULT
	    OUTPUT_VARIABLE LGIT_DESCRIBE_OUTPUT
	    ERROR_VARIABLE LGIT_DESCRIBE_ERROR
	    OUTPUT_STRIP_TRAILING_WHITESPACE
	    )
	  MESSAGE(STATUS "LGIT_DESCRIBE_OUTPUT:" ${LGIT_DESCRIBE_OUTPUT})
	  MESSAGE(STATUS "LGIT_DESCRIBE_ERROR:" ${LGIT_DESCRIBE_ERROR})

          # From the v[0-9]+.[0-9]+.[0-9]+ version remove the v in prefix.
          STRING(REGEX REPLACE "^v" "" LPROJECT_RELEASE_VERSION "${LGIT_DESCRIBE_OUTPUT}")   

          # Considers the file *.release.version
          SET(file_release_version "${PROJECT_SOURCE_DIR}/debian/${PROJECT_NAME}.release.version")

          MESSAGE(STATUS "file_release_version: ${file_release_version}")
          MESSAGE(STATUS "Everything sounds great: ${LPROJECT_RELEASE_VERSION}")
          # If this is not a git version.
          IF(LPROJECT_RELEASE_VERSION STREQUAL "" )
            # If the file exists
            MESSAGE(STATUS "Read the release version file")
            IF(EXISTS ${file_release_version})
              # Use it. This is the release version.
              FILE(STRINGS ${file_release_version} LPROJECT_RELEASE_VERSION)
            ENDIF(EXISTS ${file_release_version})
          # if this is
          ELSE(LPROJECT_RELEASE_VERSION STREQUAL "")
            # Then create the containing the release version.
            MESSAGE(STATUS "Create the release version file")
            FILE(WRITE ${file_release_version} "${LPROJECT_RELEASE_VERSION}")

          ENDIF(LPROJECT_RELEASE_VERSION STREQUAL "" )

          SET(install_file_name_src "debian/lib${PROJECT_NAME}${LPROJECT_RELEASE_VERSION}.install.cmake")
	 
	  MESSAGE(STATUS "install_file_name_src :" ${install_file_name_src})
	  IF(EXISTS ${PROJECT_SOURCE_DIR}/${install_file_name_src})
  	    SET(install_file_name_dest "debian/lib${PROJECT_NAME}${LPROJECT_RELEASE_VERSION}.install")
	    EXECUTE_PROCESS(
	        COMMAND cp ${install_file_name_src} ${install_file_name_dest}
	        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	        )

	    MESSAGE(STATUS "install_file_name :" ${install_file_name_dest})
            FILE(APPEND ${install_file_name_dest}
	      "/etc/ld.so.conf.d/*\n"
	    )
	    # Create the file to be installed.
	    SET(install_file_name "debian/lib${PROJECT_NAME}.conf")
	      FILE(WRITE ${install_file_name}
	      "${CMAKE_INSTALL_PREFIX}/lib"
	    )
	    MESSAGE(STATUS "install_file_name :" ${install_file_name})
	    INSTALL(FILES ${install_file_name} DESTINATION /etc/ld.so.conf.d )

	  ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/${install_file_name_src})
       ENDIF(${PROJECT_NAME}_IS_SHARED_LIBRARY STREQUAL "SHARED_LIBRARY")
     ENDIF(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/debian)
   ENDIF(UNIX)
 ENDIF(BUILDING_DEBIAN_PACKAGE)
ENDMACRO(_SETUP_DEBIAN)

# _SETUP_PROJECT_DEB
# -------------------
#
# Add a deb target to generate a Debian package using
# git-buildpackage (Linux specific).
#

MACRO(_SETUP_PROJECT_DEB)
  IF(UNIX AND NOT APPLE)
  ADD_CUSTOM_TARGET(deb-src
    COMMAND
    git-buildpackage
    --git-debian-branch=debian
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating source Debian package..."
    )
  ADD_CUSTOM_TARGET(deb
    COMMAND
    git-buildpackage
    --git-debian-branch=debian --git-builder="debuild -S -i.git -I.git"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating Debian package..."
    )
  ENDIF(UNIX AND NOT APPLE)
ENDMACRO(_SETUP_PROJECT_DEB)

