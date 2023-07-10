# Copyright (C) 2008-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA.
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

FUNCTION(_COMPUTE_VERSION_FROM_DOT_VERSION_FILE)
  IF(EXISTS ${PROJECT_SOURCE_DIR}/.version)
    # Yes, use it. This is a stable version.
    FILE(STRINGS .version _PROJECT_VERSION)
    SET(PROJECT_VERSION ${_PROJECT_VERSION} PARENT_SCOPE)
    SET(PROJECT_STABLE TRUE PARENT_SCOPE)
    MESSAGE(STATUS "Package version (.version): ${_PROJECT_VERSION}")
  ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/.version)
ENDFUNCTION()

FUNCTION(_COMPUTE_VERSION_FROM_GIT_DESCRIBE)
  FIND_PROGRAM(GIT git)
  IF(GIT)
    ####################################################################
    # Check whether the repository is shallow or not
    EXECUTE_PROCESS(COMMAND ${GIT} rev-parse --git-dir
                    OUTPUT_VARIABLE GIT_PROJECT_DIR
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    SET(GIT_PROJECT_DIR "${PROJECT_SOURCE_DIR}/${GIT_PROJECT_DIR}")
    IF(IS_DIRECTORY "${GIT_PROJECT_DIR}/shallow")
      SET(IS_SHALLOW TRUE)
    ELSE(IS_DIRECTORY "${GIT_PROJECT_DIR}/shallow")
      SET(IS_SHALLOW FALSE)
    ENDIF(IS_DIRECTORY "${GIT_PROJECT_DIR}/shallow")
    IF(IS_SHALLOW)
      #EXECUTE_PROCESS(COMMAND ${GIT} fetch --unshallow)
      MESSAGE(WARNING "It appears that your git repository is a shallow copy, meaning that the history has been truncated\n.
                      Please consider updating your git repository with `git fetch --unshallow` in order to download the full history with tags to recover the current release version.") 
    ENDIF(IS_SHALLOW)
    ####################################################################

    ####################################################################
    # Run describe: search for *signed* tags starting with v, from the HEAD and
    # display only the first four characters of the commit id.
    EXECUTE_PROCESS(
      COMMAND ${GIT} describe --tags --abbrev=4 --match=v* HEAD
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      RESULT_VARIABLE GIT_DESCRIBE_RESULT
      OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT
      ERROR_VARIABLE GIT_DESCRIBE_ERROR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )

    # Run diff-index to check whether the tree is clean or not.
    EXECUTE_PROCESS(
      COMMAND ${GIT} diff-index --name-only HEAD
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      RESULT_VARIABLE GIT_DIFF_INDEX_RESULT
      OUTPUT_VARIABLE GIT_DIFF_INDEX_OUTPUT
      ERROR_VARIABLE GIT_DIFF_INDEX_ERROR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )

    # Check if the tree is clean.
    IF(GIT_DIFF_INDEX_RESULT OR GIT_DIFF_INDEX_OUTPUT)
      SET(PROJECT_DIRTY TRUE PARENT_SCOPE)
    ENDIF()

    # Check if git describe worked and store the returned version number.
    IF(NOT GIT_DESCRIBE_RESULT)
      # Get rid of the tag prefix to generate the final version.
      STRING(REGEX REPLACE "^v" "" _PROJECT_VERSION "${GIT_DESCRIBE_OUTPUT}")

      # Append dirty if the project is dirty.
      IF(DEFINED PROJECT_DIRTY)
        SET(_PROJECT_VERSION "${_PROJECT_VERSION}-dirty")
      ENDIF()

      IF(_PROJECT_VERSION)
        SET(PROJECT_VERSION ${_PROJECT_VERSION} PARENT_SCOPE)
      ENDIF()

      # If there is a dash in the version number, it is an unstable release,
      # otherwise it is a stable release.
      # I.e. 1.0, 2, 0.1.3 are stable but 0.2.4-1-dg43 is unstable.
      STRING(REGEX MATCH "-" PROJECT_STABLE "${_PROJECT_VERSION}")
      IF(NOT PROJECT_STABLE STREQUAL -)
        SET(PROJECT_STABLE TRUE PARENT_SCOPE)
      ELSE()
        SET(PROJECT_STABLE FALSE PARENT_SCOPE)
      ENDIF()

      MESSAGE(STATUS "Package version (git describe): ${_PROJECT_VERSION}")
    ENDIF()
    ####################################################################
  ENDIF()
ENDFUNCTION()

FUNCTION(_COMPUTE_VERSION_FROM_ROS_PACKAGE_XML_FILE)
  IF(EXISTS ${PROJECT_SOURCE_DIR}/package.xml)
    FILE(READ "${PROJECT_SOURCE_DIR}/package.xml" PACKAGE_XML)
    EXECUTE_PROCESS(COMMAND cat "${PROJECT_SOURCE_DIR}/package.xml"
      COMMAND grep <version
      COMMAND cut -f2 -d >
      COMMAND cut -f1 -d <
      OUTPUT_STRIP_TRAILING_WHITESPACE
      #COMMAND_ECHO STDOUT
      OUTPUT_VARIABLE PACKAGE_XML_VERSION)
    IF(NOT "${PACKAGE_XML_VERSION}" STREQUAL "")
      SET(PROJECT_VERSION ${PACKAGE_XML_VERSION} PARENT_SCOPE)
    ENDIF(NOT "${PACKAGE_XML_VERSION}" STREQUAL "")
    MESSAGE(STATUS "Package version (ROS package.xml): ${PACKAGE_XML_VERSION}")
  ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/package.xml)
ENDFUNCTION()

#.rst:
# .. ifmode:: user
#
#   .. variable:: PROJECT_VERSION_COMPUTATION_METHODS
#
#    List of methods used to compute the version number.
#    Possible values are:
#
#    - *DOT_VERSION_FILE*:
#
#        If a .version file exists, interpret its content as the project version.
#
#    - *GIT_DESCRIBE*
#
#        ``git describe`` is used to retrieve the version number
#        (see 'man git-describe'). This tool generates a version number from the git
#        history. The version number follows this pattern ``TAG[-N-SHA1][-dirty]``,
#        where:
#      
#        - ``TAG``: last matching tag (i.e. last signed tag starting with v, i.e. v0.1)
#        - ``N``: number of commits since the last maching tag
#        - ``SHA1``: sha1 of the current commit
#        - ``-dirty``: added if the workig directory is dirty (there is some uncommitted
#          changes).
#
#        For stable releases, i.e. the current commit is a matching tag, ``-N-SHA1`` is
#        omitted. If the HEAD is on the signed tag v0.1, the version number will be
#        0.1.
#    
#        If the HEAD is two commits after v0.5 and the last commit is 034f6d...
#        The version number will be:
#    
#        - ``0.5-2-034f`` if there is no uncommitted changes,
#        - ``0.5-2-034f-dirty`` if there is some uncommitted changes.
#
#    - *ROS_PACKAGE_XML_FILE*
#
#        If a package.xml file exists, interpret its content as a ROS package.xml file
#        and extract the project version from its version tag.
#
#    .. note::
#
#      To safely compute the project version, you may consider the following cases:
#
#      - the software is retrieved through a tarball which does not contain the ``.git``
#        directory. Hence, there is no way to search in the Git history to generate
#        the version number.
#
#      - the softwares is retrieved through by clone a distant repository. In this case,
#        the history may not be complete (shallow clone), thus the *GIT_DESCRIBE* method may fail.
#
#      - the *DOT_VERSION_FILE* and *ROS_PACKAGE_XML_FILE* will always work but
#        forces the version number to be in the git tags and hardcoded in a file.

#.rst:
# .. ifmode:: internal
#
#   .. command:: VERSION_COMPUTE
#
#    Deduce the version number using the method as requested by
#    :cmake:variable:`PROJECT_VERSION_COMPUTATION_METHODS`.
#    The methods are called in order until one sets the variable ``PROJECT_VERSION``.
#
#    If `PROJECT_VERSION`` is already set, this macro does nothing.
#
MACRO(VERSION_COMPUTE)
  SET(PROJECT_STABLE False)

  IF("${PROJECT_SOURCE_DIR}" STREQUAL "")
    SET(PROJECT_SOURCE_DIR "${PROJECT_JRL_CMAKE_MODULE_DIR}/..")
  ENDIF()
  IF(NOT DEFINED PROJECT_VERSION_COMPUTATION_METHODS)
    LIST(APPEND PROJECT_VERSION_COMPUTATION_METHODS "ROS_PACKAGE_XML_FILE" "DOT_VERSION_FILE" "GIT_DESCRIBE")
  ENDIF()

  FOREACH(_computation_method ${PROJECT_VERSION_COMPUTATION_METHODS})
    IF(NOT PROJECT_VERSION)
      IF    (${_computation_method} STREQUAL "DOT_VERSION_FILE")
        _COMPUTE_VERSION_FROM_DOT_VERSION_FILE()
      ELSEIF(${_computation_method} STREQUAL "GIT_DESCRIBE")
        _COMPUTE_VERSION_FROM_GIT_DESCRIBE()
      ELSEIF(${_computation_method} STREQUAL "ROS_PACKAGE_XML_FILE")
        _COMPUTE_VERSION_FROM_ROS_PACKAGE_XML_FILE()
      ELSE  ()
        MESSAGE(AUTHOR_WARNING "${_computation_method} is not a valid method to compute the project version.")
      ENDIF()
    ENDIF(NOT PROJECT_VERSION)
  ENDFOREACH()
  IF(NOT PROJECT_VERSION)
    # set a default, ref https://github.com/jrl-umi3218/jrl-cmakemodules/issues/381
    SET(PROJECT_VERSION 0.0.0)
  ENDIF(NOT PROJECT_VERSION)
  
  # Set PROJECT_VERSION_{MAJOR,MINOR,PATCH} variables
  IF(PROJECT_VERSION)
    # Compute the major, minor and patch version of the project
    IF(NOT DEFINED PROJECT_VERSION_MAJOR AND
       NOT DEFINED PROJECT_VERSION_MINOR AND
       NOT DEFINED PROJECT_VERSION_PATCH)
     SPLIT_VERSION_NUMBER(${PROJECT_VERSION}
        PROJECT_VERSION_MAJOR
        PROJECT_VERSION_MINOR
        PROJECT_VERSION_PATCH)
    ENDIF()
  ENDIF()

ENDMACRO()

MACRO(SPLIT_VERSION_NUMBER VERSION
    VERSION_MAJOR_VAR
    VERSION_MINOR_VAR
    VERSION_PATCH_VAR)
  # Compute the major, minor and patch version of the project
  IF(${VERSION} MATCHES UNKNOWN)
    SET(${VERSION_MAJOR_VAR} UNKNOWN)
    SET(${VERSION_MINOR_VAR} UNKNOWN)
    SET(${VERSION_PATCH_VAR} UNKNOWN)
  ELSE()
    # Extract the version from PROJECT_VERSION
    string(REGEX REPLACE "-.*$" "" _PROJECT_VERSION_LIST "${VERSION}")
    string(REPLACE "." ";" _PROJECT_VERSION_LIST "${_PROJECT_VERSION_LIST}")
    list(LENGTH _PROJECT_VERSION_LIST SIZE)
    IF(${SIZE} GREATER 0)
      list(GET _PROJECT_VERSION_LIST 0 ${VERSION_MAJOR_VAR})
    ENDIF()
    IF(${SIZE} GREATER 1)
      list(GET _PROJECT_VERSION_LIST 1 ${VERSION_MINOR_VAR})
    ENDIF()
    IF(${SIZE} GREATER 2)
      list(GET _PROJECT_VERSION_LIST 2 ${VERSION_PATCH_VAR})
    ENDIF()
  ENDIF()
ENDMACRO()
