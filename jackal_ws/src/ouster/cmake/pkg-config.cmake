# Copyright (C) 2008-2020 LAAS-CNRS, JRL AIST-CNRS INRIA.
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

INCLUDE(${CMAKE_CURRENT_LIST_DIR}/shared-library.cmake)

FIND_PACKAGE(PkgConfig)

# For CMake >= 3.12, this can be replace by
# list(JOIN ${${var_in}} ${sep} out)
function(_list_join var_in sep var_out)
  if(CMAKE_VERSION VERSION_GREATER 3.12)
    list(JOIN ${var_in} ${sep} out)
  else()
    set(first TRUE)
    foreach(el ${${var_in}})
      if(first)
        set(out "${el}")
        set(first FALSE)
      else()
        set(out "${out}${sep}${el}")
      endif()
    endforeach()
  endif()
  set(${var_out} ${out} PARENT_SCOPE)
endfunction()

# Additional pkg-config variables whose value will be imported
# during the dependency check.
SET(PKG_CONFIG_ADDITIONAL_VARIABLES bindir pkglibdir datarootdir pkgdatarootdir docdir doxygendocdir)

#.rst:
# .. ifmode:: internal
#
#   .. command:: _SETUP_PROJECT_PKG_CONFIG
#
#     Prepare pkg-config pc file generation step.
#
#     This file will be named ${PROJECT_NAME}.pc,
#     or ${CUSTOM_PKG_CONFIG_FILENAME}.pc if it is defined
#
MACRO(_SETUP_PROJECT_PKG_CONFIG)
  # Pkg-config related commands.
  SET(_PKG_CONFIG_PREFIX "${CMAKE_INSTALL_PREFIX}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_EXEC_PREFIX "${_PKG_CONFIG_PREFIX}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_FULL_LIBDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_BINDIR "${CMAKE_INSTALL_FULL_BINDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_PKGLIBDIR "${CMAKE_INSTALL_FULL_PKGLIBDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_FULL_INCLUDEDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_DATAROOTDIR "${CMAKE_INSTALL_FULL_DATAROOTDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_PKGDATAROOTDIR "${CMAKE_INSTALL_FULL_DATADIR}" CACHE INTERNAL "")
  IF(INSTALL_DOCUMENTATION)
    SET(_PKG_CONFIG_DOCDIR "${CMAKE_INSTALL_FULL_DOCDIR}" CACHE INTERNAL "")
    SET(_PKG_CONFIG_DOXYGENDOCDIR "${_PKG_CONFIG_DOCDIR}/doxygen-html" CACHE INTERNAL "")
  ELSE(INSTALL_DOCUMENTATION)
    SET(_PKG_CONFIG_DOCDIR "" CACHE INTERNAL "")
    SET(_PKG_CONFIG_DOXYGENDOCDIR "" CACHE INTERNAL "")
  ENDIF(INSTALL_DOCUMENTATION)

  IF(DEFINED PROJECT_DEBUG_POSTFIX)
    IF(DEFINED CMAKE_CONFIGURATION_TYPES)
      SET(_PKG_CONFIG_PROJECT_NAME_NOPOSTFIX "${PROJECT_NAME}" CACHE INTERNAL "")
      SET(_PKG_CONFIG_PROJECT_NAME "${PROJECT_NAME}${PKGCONFIG_POSTFIX}" CACHE INTERNAL "")
    ELSE()
      SET(_PKG_CONFIG_PROJECT_NAME "${PROJECT_NAME}${PKGCONFIG_POSTFIX}" CACHE INTERNAL "")
    ENDIF()
  ELSE()
    SET(_PKG_CONFIG_PROJECT_NAME "${PROJECT_NAME}" CACHE INTERNAL "")
  ENDIF()
  SET(_PKG_CONFIG_DESCRIPTION "${PROJECT_DESCRIPTION}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_URL "${PROJECT_URL}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_VERSION "${PROJECT_VERSION}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_REQUIRES "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_REQUIRES_DEBUG "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_REQUIRES_OPTIMIZED "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_COMPILE_TIME_REQUIRES "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CONFLICTS "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS_DEBUG "${LIBDIR_KW}\${libdir}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS_OPTIMIZED "${LIBDIR_KW}\${libdir}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS_PRIVATE "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CFLAGS "-I\${includedir}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CFLAGS_DEBUG "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CFLAGS_OPTIMIZED "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_FILENAME "${PROJECT_NAME}.pc" CACHE INTERNAL "")

  SET(PKG_CONFIG_EXTRA "")

  # Where to install the pkg-config file?
  SET(_PKG_CONFIG_DIR "${_PKG_CONFIG_LIBDIR}/pkgconfig" CACHE INTERNAL "")

  # Watch variables.
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    _PKG_CONFIG_FOUND
    PKG_CONFIG_EXECUTABLE
    _PKG_CONFIG_PREFIX
    _PKG_CONFIG_EXEC_PREFIX
    _PKG_CONFIG_LIBDIR
    _PKG_CONFIG_BINDIR
    _PKG_CONFIG_PKGLIBDIR
    _PKG_CONFIG_INCLUDEDIR
    _PKG_CONFIG_DATAROOTDIR
    _PKG_CONFIG_PKGDATAROOTDIR
    _PKG_CONFIG_DOCDIR
    _PKG_CONFIG_DOXYGENDOCDIR
    _PKG_CONFIG_PROJECT_NAME
    _PKG_CONFIG_DESCRIPTION
    _PKG_CONFIG_URL
    _PKG_CONFIG_VERSION
    _PKG_CONFIG_REQUIRES
    _PKG_CONFIG_REQUIRES_DEBUG
    _PKG_CONFIG_REQUIRES_OPTIMIZED
    _PKG_CONFIG_COMPILE_TIME_REQUIRES
    _PKG_CONFIG_CONFLICTS
    _PKG_CONFIG_LIBS
    _PKG_CONFIG_LIBS_DEBUG
    _PKG_CONFIG_LIBS_OPTIMIZED
    _PKG_CONFIG_LIBS_PRIVATE
    _PKG_CONFIG_CFLAGS
    _PKG_CONFIG_CFLAGS_DEBUG
    _PKG_CONFIG_CFLAGS_OPTIMIZED
    _PKG_CONFIG_FILENAME
    PKG_CONFIG_EXTRA
    )
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG)


# _SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG
# ----------------------------------
#
# Post-processing of the pkg-config step.
#
# The pkg-config file has to be generated at the end to allow end-user
# defined variables replacement.
#
# This macro adds _PKG_CONFIG_LIBS_DEBUG to _PKG_CONFIG_LIBS and
# _PKGCONFIG_CFLAGS_DEBUG to _PKG_CONFIG_CFLAGS
#
MACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG)
  # Setup altered variables
  SET(TEMP_CFLAGS ${_PKG_CONFIG_CFLAGS})
  SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS_DEBUG} ${_PKG_CONFIG_CFLAGS}")
  SET(TEMP_LIBS ${_PKG_CONFIG_LIBS})
  SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS_DEBUG} ${_PKG_CONFIG_LIBS}")
  SET(TEMP_REQUIRES ${_PKG_CONFIG_REQUIRES})
  IF(_PKG_CONFIG_REQUIRES_DEBUG)
    LIST(APPEND _PKG_CONFIG_REQUIRES "${_PKG_CONFIG_REQUIRES_DEBUG}")
  ENDIF()
  _list_join(_PKG_CONFIG_REQUIRES ", " _PKG_CONFIG_REQUIRES_LIST)
  CONFIGURE_FILE(
    "${PROJECT_JRL_CMAKE_MODULE_DIR}/pkg-config.pc.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}${PKGCONFIG_POSTFIX}.pc"
    )
  # Restore altered variables
  SET(_PKG_CONFIG_CFLAGS ${TEMP_CFLAGS})
  SET(_PKG_CONFIG_LIBS ${TEMP_LIBS})
  SET(_PKG_CONFIG_REQUIRES ${TEMP_REQUIRES})

  INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}${PKGCONFIG_POSTFIX}.pc"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG)

# _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED
# ----------------------------------
#
# Post-processing of the pkg-config step.
#
# The pkg-config file has to be generated at the end to allow end-user
# defined variables replacement.
#
# This macro adds _PKG_CONFIG_LIBS_OPTIMIZED to _PKG_CONFIG_LIBS and
# _PKGCONFIG_CFLAGS_OPTIMIZED to _PKG_CONFIG_CFLAGS
#
MACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED)
  # Setup altered variables
  SET(TEMP_PROJECT_NAME ${_PKG_CONFIG_PROJECT_NAME})
  SET(_PKG_CONFIG_PROJECT_NAME ${_PKG_CONFIG_PROJECT_NAME_NOPOSTFIX})
  SET(TEMP_CFLAGS ${_PKG_CONFIG_CFLAGS})
  SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS_OPTIMIZED} ${_PKG_CONFIG_CFLAGS}")
  SET(TEMP_LIBS ${_PKG_CONFIG_LIBS})
  SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS_OPTIMIZED} ${_PKG_CONFIG_LIBS}")
  SET(TEMP_REQUIRES ${_PKG_CONFIG_REQUIRES})
  IF(_PKG_CONFIG_REQUIRES_OPTIMIZED)
    LIST(APPEND _PKG_CONFIG_REQUIRES "${_PKG_CONFIG_REQUIRES_OPTIMIZED}")
  ENDIF()
  _list_join(_PKG_CONFIG_REQUIRES ", " _PKG_CONFIG_REQUIRES_LIST)
  IF(DEFINED CUSTOM_PKG_CONFIG_FILENAME)
    SET(_PKG_CONFIG_FILENAME "${CUSTOM_PKG_CONFIG_FILENAME}.pc" CACHE INTERNAL "")
  ENDIF(DEFINED CUSTOM_PKG_CONFIG_FILENAME)
  # Generate the pkg-config file.
  CONFIGURE_FILE(
    "${PROJECT_JRL_CMAKE_MODULE_DIR}/pkg-config.pc.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${_PKG_CONFIG_FILENAME}"
    )
  # Restore altered variables
  SET(_PKG_CONFIG_PROJECT_NAME ${TEMP_PROJECT_NAME})
  SET(_PKG_CONFIG_CFLAGS ${TEMP_CFLAGS})
  SET(_PKG_CONFIG_LIBS ${TEMP_LIBS})
  SET(_PKG_CONFIG_REQUIRES ${TEMP_REQUIRES})

  INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${_PKG_CONFIG_FILENAME}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED)

# _SETUP_PROJECT_PKG_CONFIG_FINALIZE
# ----------------------------------
#
# Post-processing of the pkg-config step.
#
# The pkg-config file has to be generated at the end to allow end-user
# defined variables replacement.
#
MACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE)
  # Single build type generator
  IF(DEFINED CMAKE_BUILD_TYPE)
    STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
    IF(${cmake_build_type} MATCHES debug)
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG()
    ELSE()
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED()
    ENDIF()
  # Multiple build types generator
  ELSE()
    IF(DEFINED PROJECT_DEBUG_POSTFIX)
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG()
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED()
    ELSE()
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED()
    ENDIF()
  ENDIF()
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE)

# _PARSE_PKG_CONFIG_STRING (PKG_CONFIG_STRING _PKG_LIB_NAME_VAR _PKG_PREFIX_VAR _PKG_CONFIG_STRING_NOSPACE_VAR)
# ----------------------------------------------------------
#
# Retrieve from the pkg-config string:
# - the library name,
# - the prefix used for CMake variable names,
# - a variant without spaces around the operator (if there is an operator),
#   as expected by cmake's CHECK_PKG_MODULE.
# . For instance, `my-package > 0.4` results in
# - _PKG_LIB_NAME_VAR <- my-package
# - _PKG_PREFIX_VAR <- MY_PACKAGE
# - _PKG_CONFIG_STRING_NOSPACE_VAR <- `my-package>0.4`
# `my-package` results in
# - _PKG_LIB_NAME_VAR <- my-package
# - _PKG_PREFIX_VAR <- MY_PACKAGE
# - _PKG_CONFIG_STRING_NOSPACE_VAR <- `my-package`
MACRO(_PARSE_PKG_CONFIG_STRING PKG_CONFIG_STRING _PKG_LIB_NAME_VAR _PKG_PREFIX_VAR _PKG_CONFIG_NOSPACE_VAR)
  # Decompose the equation
  STRING(REGEX MATCH "([^ ]+)( (>|>=|=|<=|<) (.*))?" _UNUSED "${PKG_CONFIG_STRING}")
  # Reconstruct the equation, without the space around the operator
  SET(${_PKG_CONFIG_NOSPACE_VAR} "${CMAKE_MATCH_1}${CMAKE_MATCH_3}${CMAKE_MATCH_4}")
  # The left part of the equation is the package name
  SET(${_PKG_LIB_NAME_VAR} "${CMAKE_MATCH_1}")
  # Transform it into a valid variable prefix.
  # 1. replace invalid characters into underscores.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" ${_PKG_PREFIX_VAR} "${${_PKG_LIB_NAME_VAR}}")
  # 2. make it uppercase.
  STRING(TOUPPER "${${_PKG_PREFIX_VAR}}" ${_PKG_PREFIX_VAR})
ENDMACRO()

# ADD_DEPENDENCY(PREFIX P_REQUIRED COMPILE_TIME_ONLY PKGCONFIG_STRING)
# ------------------------------------------------
#
# Check for a dependency using pkg-config. Fail if the package cannot
# be found.
#
# P_REQUIRED : if set to 1 the package is required, otherwise it consider
#              as optional.
#              WARNING for optional package:
#              if the package is detected its compile
#              and linking options are still put in the required fields
#              of the generated pc file. Indeed from the binary viewpoint
#              the package becomes required.
#
# COMPILE_TIME_ONLY : if set to 1, the package is only requiered at compile time and won't
#                     appear as a dependency inside the *.pc file.
#
# PKG_CONFIG_STRING	: string passed to pkg-config to check the version.
#			  Typically, this string looks like:
#                         ``my-package >= 0.5''
#
MACRO(ADD_DEPENDENCY P_REQUIRED COMPILE_TIME_ONLY PKG_CONFIG_STRING PKG_CONFIG_DEBUG_STRING)
  _PARSE_PKG_CONFIG_STRING ("${PKG_CONFIG_STRING}" LIBRARY_NAME PREFIX PKG_CONFIG_STRING_NOSPACE)
  IF(NOT ${PKG_CONFIG_DEBUG_STRING} STREQUAL "")
    _PARSE_PKG_CONFIG_STRING ("${PKG_CONFIG_DEBUG_STRING}" LIBRARY_DEBUG_NAME ${PREFIX}_DEBUG PKG_CONFIG_DEBUG_STRING_NOSPACE)
  ENDIF()

  # Force redetection each time CMake is launched.
  # Rationale: these values are *NEVER* manually set, so information is never
  # lost by overriding them. Moreover, changes in the pkg-config files are
  # not seen as long as the cache is not destroyed, even if the .pc file
  # is changed. This is a BAD behavior.
  SET(${PREFIX}_FOUND 0)
  IF(DEFINED ${PREFIX}_DEBUG)
    SET(${PREFIX}_DEBUG_FOUND 0)
  ENDIF()

  # This makes the debug dependency optional when building in release and
  # vice-versa, this only applies to single build type generators
  SET(PP_REQUIRED ${P_REQUIRED}) # Work-around macro limitation
  IF(DEFINED ${PREFIX}_DEBUG)
    SET(P_DEBUG_REQUIRED ${P_REQUIRED})
    IF(${P_REQUIRED})
      # Single build type generators
      IF(DEFINED CMAKE_BUILD_TYPE)
        STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
        IF("${cmake_build_type}" MATCHES "debug")
          SET(PP_REQUIRED 0)
        ELSE()
          SET(P_DEBUG_REQUIRED 0)
        ENDIF()
      ENDIF()
    ENDIF()
  ENDIF()

  # Search for the package.
  IF(${PP_REQUIRED})
    MESSAGE(STATUS "${PKG_CONFIG_STRING} is required.")
    PKG_CHECK_MODULES("${PREFIX}" REQUIRED "${PKG_CONFIG_STRING_NOSPACE}")
  ELSE(${PP_REQUIRED})
    MESSAGE(STATUS "${PKG_CONFIG_STRING} is optional.")
    PKG_CHECK_MODULES("${PREFIX}" "${PKG_CONFIG_STRING_NOSPACE}")
  ENDIF(${PP_REQUIRED})

  # Search for the debug package
  IF(DEFINED ${PREFIX}_DEBUG)
    IF(${P_DEBUG_REQUIRED})
      MESSAGE(STATUS "${PKG_CONFIG_DEBUG_STRING} is required")
      PKG_CHECK_MODULES("${PREFIX}_DEBUG" REQUIRED "${PKG_CONFIG_DEBUG_STRING_NOSPACE}")
    ELSE(${P_DEBUG_REQUIRED})
      MESSAGE(STATUS "${PKG_CONFIG_DEBUG_STRING} is optional")
      PKG_CHECK_MODULES("${PREFIX}_DEBUG" "${PKG_CONFIG_DEBUG_STRING_NOSPACE}")
    ENDIF(${P_DEBUG_REQUIRED})
  ENDIF()

  # Fix for ld >= 2.24.90: -l:/some/absolute/path.so is no longer supported.
  # See shared-library.cmake.
  IF(UNIX AND NOT ${LD_VERSION} VERSION_LESS "2.24.90")
    STRING(REPLACE ":/" "/" "${PREFIX}_LIBRARIES" "${${PREFIX}_LIBRARIES}")
    STRING(REPLACE "-l:/" "/" "${PREFIX}_LDFLAGS" "${${PREFIX}_LDFLAGS}")

    IF(DEFINED ${PREFIX}_DEBUG)
      STRING(REPLACE ":/" "/" "${PREFIX}_DEBUG_LIBRARIES" "${${PREFIX}_DEBUG_LIBRARIES}")
      STRING(REPLACE "-l:/" "/" "${PREFIX}_DEBUG_LDFLAGS" "${${PREFIX}_DEBUG_LDFLAGS}")
    ENDIF()
  ENDIF()

  # Watch variables.
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    ${PREFIX}_FOUND
    ${PREFIX}_LIBRARIES
    ${PREFIX}_LIBRARY_DIRS
    ${PREFIX}_LDFLAGS
    ${PREFIX}_LDFLAGS_OTHER
    ${PREFIX}_INCLUDE_DIRS
    ${PREFIX}_CFLAGS
    ${PREFIX}_CFLAGS_OTHER
    ${PREFIX}
    ${PREFIX}_STATIC
    ${PREFIX}_VERSION
    ${PREFIX}_PREFIX
    ${PREFIX}_INCLUDEDIR
    ${PREFIX}_LIBDIR
    ${PREFIX}_PKGLIBDIR
    ${PREFIX}_BINDIR
    ${PREFIX}_DATAROOTDIR
    ${PREFIX}_PKGDATAROOTDIR
    ${PREFIX}_DOCDIR
    ${PREFIX}_DOXYGENDOCDIR
    )
  IF(DEFINED ${PREFIX}_DEBUG)
    LIST(APPEND LOGGING_WATCHED_VARIABLES
      ${PREFIX}_DEBUG_FOUND
      ${PREFIX}_DEBUG_LIBRARIES
      ${PREFIX}_DEBUG_LIBRARY_DIRS
      ${PREFIX}_DEBUG_LDFLAGS
      ${PREFIX}_DEBUG_LDFLAGS_OTHER
      ${PREFIX}_DEBUG_INCLUDE_DIRS
      ${PREFIX}_DEBUG_CFLAGS
      ${PREFIX}_DEBUG_CFLAGS_OTHER
      ${PREFIX}_DEBUG
      ${PREFIX}_DEBUG_STATIC
      ${PREFIX}_DEBUG_VERSION
      ${PREFIX}_DEBUG_PREFIX_DEBUG
      ${PREFIX}_DEBUG_INCLUDEDIR
      ${PREFIX}_DEBUG_LIBDIR
      ${PREFIX}_DEBUG_PKGLIBDIR
      ${PREFIX}_DEBUG_BINDIR
      ${PREFIX}_DEBUG_DATAROOTDIR
      ${PREFIX}_DEBUG_PKGDATAROOTDIR
      ${PREFIX}_DEBUG_DOCDIR
      ${PREFIX}_DEBUG_DOXYGENDOCDIR
      )
  ENDIF()

  # Get the values of additional variables.
  FOREACH(VARIABLE ${PKG_CONFIG_ADDITIONAL_VARIABLES})
    # Upper-case version of the variable for CMake variable generation.
    STRING(TOUPPER "${VARIABLE}" "VARIABLE_UC")
    EXECUTE_PROCESS(
      COMMAND "${PKG_CONFIG_EXECUTABLE}"
      "--variable=${VARIABLE}" "${LIBRARY_NAME}"
      OUTPUT_VARIABLE "${PREFIX}_${VARIABLE_UC}"
      ERROR_QUIET)
    STRING(REPLACE "\n" "" "${PREFIX}_${VARIABLE_UC}" "${${PREFIX}_${VARIABLE_UC}}")
    # Watch additional variables.
    LIST(APPEND LOGGING_WATCHED_VARIABLES ${PREFIX}_${VARIABLE_UC})
    IF(DEFINED ${PREFIX}_DEBUG)
      EXECUTE_PROCESS(
        COMMAND "${PKG_CONFIG_EXECUTABLE}"
        "--variable=${VARIABLE}" "${LIBRARY_DEBUG_NAME}"
        OUTPUT_VARIABLE "${PREFIX}_DEBUG_${VARIABLE_UC}"
        ERROR_QUIET)
      STRING(REPLACE "\n" "" "${PREFIX}_DEBUG_${VARIABLE_UC}" "${${PREFIX}_DEBUG_${VARIABLE_UC}}")
      LIST(APPEND LOGGING_WATCHED_VARIABLES ${PREFIX}_DEBUG_${VARIABLE_UC})
    ENDIF()
  ENDFOREACH(VARIABLE)

  #FIXME: spaces are replaced by semi-colon by mistakes, revert the change.
  #I cannot see why CMake is doing that...
  STRING(REPLACE ";" " " PKG_CONFIG_STRING "${PKG_CONFIG_STRING}")
  IF(DEFINED ${PREFIX}_DEBUG)
    STRING(REPLACE ";" " " PKG_CONFIG_DEBUG_STRING "${PKG_CONFIG_DEBUG_STRING}")
  ENDIF()

  IF(DEFINED ${PREFIX}_DEBUG)
    IF(${${PREFIX}_FOUND})
      SET(PACKAGE_FOUND 1)
    ELSEIF(${${PREFIX}_DEBUG_FOUND})
      SET(PACKAGE_FOUND 1)
    ELSE()
      SET(PACKAGE_FOUND 0)
    ENDIF()
  ELSE()
    IF(${${PREFIX}_FOUND})
      SET(PACKAGE_FOUND 1)
    ELSE()
      SET(PACKAGE_FOUND 0)
    ENDIF()
  ENDIF()

  # Add the package to the dependency list if found and if dependency
  # is triggered not only for documentation
  IF(${PACKAGE_FOUND})
    IF(NOT ${COMPILE_TIME_ONLY})
      IF(DEFINED PROJECT_DEBUG_POSTFIX AND DEFINED ${PREFIX}_DEBUG)
        _ADD_TO_LIST_IF_NOT_PRESENT( _PKG_CONFIG_REQUIRES_DEBUG "${PKG_CONFIG_DEBUG_STRING}")
        _ADD_TO_LIST_IF_NOT_PRESENT( _PKG_CONFIG_REQUIRES_OPTIMIZED "${PKG_CONFIG_STRING}")
      ELSE()
        # Warn the user in case he/she is using alternative libraries for debug but no postfix
        IF(NOT DEFINED PROJECT_DEBUG_POSTFIX AND DEFINED ${PREFIX}_DEBUG)
          MESSAGE(AUTHOR_WARNING
            "You are linking with different libraries in debug mode but the
             generated .pc cannot reflect that, it will default to release flags. Consider
             setting PROJECT_DEBUG_POSTFIX to generate different libraries and pc files in
             debug mode.")
        ENDIF()
        _ADD_TO_LIST_IF_NOT_PRESENT( _PKG_CONFIG_REQUIRES "${PKG_CONFIG_STRING}")
      ENDIF()
    ELSE()
      _ADD_TO_LIST_IF_NOT_PRESENT( _PKG_CONFIG_COMPILE_TIME_REQUIRES "${PKG_CONFIG_STRING}")
    ENDIF()
  ENDIF()

  # Add the package to the cmake dependency list
  # if cpack has been included.
  # This is likely to disappear when Ubuntu 8.04 will
  # disappear.
  IF(COMMAND ADD_CMAKE_DEPENDENCY)
    ADD_CMAKE_DEPENDENCY(${PKG_CONFIG_STRING})
  ENDIF(COMMAND ADD_CMAKE_DEPENDENCY)

  IF(${${PREFIX}_FOUND})
   MESSAGE(STATUS
    "Pkg-config module ${LIBRARY_NAME} v${${PREFIX}_VERSION}"
    " has been detected with success.")
  ENDIF()
  IF(DEFINED ${PREFIX}_DEBUG AND "${${PREFIX}_DEBUG_FOUND}")
   MESSAGE(STATUS
    "Pkg-config module ${LIBRARY_DEBUG_NAME} v${${PREFIX}_DEBUG_VERSION}"
    " has been detected with success.")
  ENDIF()

ENDMACRO(ADD_DEPENDENCY)

#.rst:
# .. ifmode:: internal
#
#   .. command:: _GET_PKG_CONFIG_DEBUG_STRING
#
#     Used in ADD_*_DEPENDENCY to get the PKG_CONFIG_DEBUG_STRING argument.  On
#     WIN32, if the string is absent but PROJECT_DEBUG_POSTFIX is set, attempts
#     to locate a package matching the PROJECT_DEBUG_POSTFIX for debug builds.
#
MACRO(_GET_PKG_CONFIG_DEBUG_STRING PKG_CONFIG_STRING)
  SET(PKG_CONFIG_DEBUG_STRING "")
  FOREACH(ARG ${ARGN})
    SET(PKG_CONFIG_DEBUG_STRING ${ARG})
  ENDFOREACH()
  IF(WIN32 AND DEFINED PROJECT_DEBUG_POSTFIX AND "${PKG_CONFIG_DEBUG_STRING}" STREQUAL "")
    _PARSE_PKG_CONFIG_STRING("${PKG_CONFIG_STRING}" LIBRARY_NAME PREFIX)
    STRING(REGEX REPLACE "${LIBRARY_NAME}" "${LIBRARY_NAME}${PROJECT_DEBUG_POSTFIX}" LIBRARY_NAME "${PKG_CONFIG_STRING}")
    PKG_CHECK_MODULES("${PREFIX}" "${LIBRARY_NAME}")
    IF(${PREFIX}_FOUND)
      SET(PKG_CONFIG_DEBUG_STRING "${LIBRARY_NAME}")
    ENDIF()
  ENDIF()
ENDMACRO(_GET_PKG_CONFIG_DEBUG_STRING)

#.rst:
# .. ifmode:: import
#
#   .. command:: ADD_REQUIRED_DEPENDENCY (PKG_CONFIG_STRING PKG_CONFIG_DEBUG_STRING)
#
#     Check for a dependency using pkg-config. Fail if the package cannot
#     be found.
#
#     :PKG_CONFIG_STRING: string passed to pkg-config to check the version.
#       Typically, this string looks like: ``my-package >= 0.5``
#
#     :PKG_CONFIG_DEBUG_STRING: (optional) string passed to pkg-config to
#       check the version. The package found this way will be used in place
#       of the first provided if the build is happening in DEBUG mode.
#       This string might look like: ``my-package_d >= 0.5``
#
#     An optional argument can be passed to define an alternate PKG_CONFIG_STRING
#     for debug builds. It should follow the same rule as PKG_CONFIG_STRING.
#
MACRO(ADD_REQUIRED_DEPENDENCY PKG_CONFIG_STRING)
  LIST (FIND _PKG_CONFIG_REQUIRES "${PKG_CONFIG_STRING}" _index)
  IF(${_index} EQUAL -1)
    _GET_PKG_CONFIG_DEBUG_STRING("${PKG_CONFIG_STRING}" ${ARGN})
    ADD_DEPENDENCY(1 0 ${PKG_CONFIG_STRING} "${PKG_CONFIG_DEBUG_STRING}")
  ELSE()
    # Already found
    message(STATUS "Package ${PKG_CONFIG_STRING} already found.")
  ENDIF()
ENDMACRO(ADD_REQUIRED_DEPENDENCY)

#.rst:
# .. ifmode:: import
#
#   .. command:: ADD_OPTIONAL_DEPENDENCY (PKG_CONFIG_STRING PKG_CONFIG_DEBUG_STRING)
#
#     Check for a dependency using pkg-config. Quiet if the package cannot
#     be found.
#
#     :PKG_CONFIG_STRING: string passed to pkg-config to check the version.
#       Typically, this string looks like: ``my-package >= 0.5``
#
#     :PKG_CONFIG_DEBUG_STRING: (optional) string passed to pkg-config to
#       check the version. The package found this way will be used in place
#       of the first provided if the build is happening in DEBUG mode.
#       This string might look like: ``my-package_d >= 0.5``
#
#     An optional argument can be passed to define an alternate PKG_CONFIG_STRING
#     for debug builds. It should follow the same rule as PKG_CONFIG_STRING.
#
MACRO(ADD_OPTIONAL_DEPENDENCY PKG_CONFIG_STRING)
  _GET_PKG_CONFIG_DEBUG_STRING("${PKG_CONFIG_STRING}" ${ARGN})
  ADD_DEPENDENCY(0 0 ${PKG_CONFIG_STRING} "${PKG_CONFIG_DEBUG_STRING}")
ENDMACRO(ADD_OPTIONAL_DEPENDENCY)

#.rst:
# .. ifmode:: import-advanced
#
#   .. command:: ADD_COMPILE_DEPENDENCY (PKGCONFIG_STRING)
#
#     Check for a dependency using pkg-config. Fail if the package cannot
#     be found.  The package won't appear as depency inside the \*.pc file
#     of the PROJECT.
#
#     :PKG_CONFIG_STRING: string passed to pkg-config to check the version.
#       Typically, this string looks like: ``my-package >= 0.5``
#
#     :PKG_CONFIG_DEBUG_STRING: (optional) string passed to pkg-config to
#       check the version. The package found this way will be used in place
#       of the first provided if the build is happening in DEBUG mode.  This
#       string might look like: ``my-package_d >= 0.5``
#
#
MACRO(ADD_COMPILE_DEPENDENCY PKG_CONFIG_STRING)
  _GET_PKG_CONFIG_DEBUG_STRING("${PKG_CONFIG_STRING}" ${ARGN})
  ADD_DEPENDENCY(1 1 ${PKG_CONFIG_STRING} "${PKG_CONFIG_DEBUG_STRING}")
ENDMACRO(ADD_COMPILE_DEPENDENCY)


#.rst:
# .. ifmode:: import-advanced
#
#   .. command:: ADD_DOC_DEPENDENCY (PKGCONFIG_STRING)
#
#     Alias for :command:`ADD_COMPILE_DEPENDENCY`
#
MACRO(ADD_DOC_DEPENDENCY PKG_CONFIG_STRING)
  ADD_COMPILE_DEPENDENCY(${PKG_CONFIG_STRING})
ENDMACRO(ADD_DOC_DEPENDENCY)

#.rst:
# .. ifmode:: export
#
#   .. command:: PKG_CONFIG_APPEND_LIBRARY_DIR (DIRS)
#
#     This macro adds library directories ``DIRS`` in a portable way into
#     the CMake file.
#
MACRO(PKG_CONFIG_APPEND_LIBRARY_DIR DIRS)
  FOREACH(DIR ${DIRS})
    IF(DIR)
      SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBDIR_KW}${DIR}" CACHE INTERNAL "")
    ENDIF(DIR)
  ENDFOREACH(DIR ${DIRS})
ENDMACRO(PKG_CONFIG_APPEND_LIBRARY_DIR DIR)

#.rst:
# .. ifmode:: export-advanced
#
#   .. command:: PKG_CONFIG_APPEND_CFLAGS_DEBUG (FLAGS)
#
#
#     This macro adds ``FLAGS`` in a portable way into the pkg-config file
#     of the debug library.
#
#     As such the macro fails if ``PROJECT_DEBUG_POSTFIX`` is not set
#
MACRO(PKG_CONFIG_APPEND_CFLAGS_DEBUG FLAGS)
  IF(NOT DEFINED PROJECT_DEBUG_POSTFIX)
    MESSAGE(FATAL_ERROR "You are trying to use PKG_CONFIG_APPEND_CFLAGS_DEBUG on a package that does not have a debug library")
  ENDIF()
  FOREACH(FLAG ${FLAGS})
    IF(FLAG)
      SET(_PKG_CONFIG_CFLAGS_DEBUG "${_PKG_CONFIG_CFLAGS_DEBUG} ${FLAG}" CACHE INTERNAL "")
    ENDIF(FLAG)
  ENDFOREACH(FLAG ${FLAGS})
ENDMACRO(PKG_CONFIG_APPEND_CFLAGS_DEBUG FLAGS)

#.rst:
# .. ifmode:: export-advanced
#
#   .. command:: PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED (FLAGS)
#
#
#     This macro adds ``FLAGS`` in a portable way into the pkg-config file
#     of the optimized library.
#
#     As such the macro fails if ``PROJECT_DEBUG_POSTFIX`` is not set
#
MACRO(PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED FLAGS)
  IF(NOT DEFINED PROJECT_DEBUG_POSTFIX)
    MESSAGE(FATAL_ERROR "You are trying to use PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED on a package that does not have a debug library")
  ENDIF()
  FOREACH(FLAG ${FLAGS})
    IF(FLAG)
      SET(_PKG_CONFIG_CFLAGS_OPTIMIZED "${_PKG_CONFIG_CFLAGS_OPTIMIZED} ${FLAG}" CACHE INTERNAL "")
    ENDIF(FLAG)
  ENDFOREACH(FLAG ${FLAGS})
ENDMACRO(PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED FLAGS)

#.rst:
# .. ifmode:: export
#
#   .. command:: PKG_CONFIG_APPEND_CFLAGS (FLAGS)
#
#     This macro adds ``FLAGS`` in a portable way into the pkg-config file.
#
MACRO(PKG_CONFIG_APPEND_CFLAGS FLAGS)
  FOREACH(FLAG ${FLAGS})
    IF(FLAG)
      SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} ${FLAG}" CACHE INTERNAL "")
    ENDIF(FLAG)
  ENDFOREACH(FLAG ${FLAGS})
ENDMACRO(PKG_CONFIG_APPEND_CFLAGS)


#.rst:
# .. ifmode:: export-advanced
#
#   .. command:: PKG_CONFIG_APPEND_LIBS_RAW (LIBS)
#
#     This macro adds raw value ``LIBS`` in the "Libs:" section of the
#     pkg-config file.
#
#     **Exception for mac OS X**
#
#       In addition to the classical static and dynamic libraries (handled
#       like unix does), mac systems can link against frameworks.
#       Frameworks are directories gathering headers, libraries, shared
#       resources...
#
#       The syntax used to link with a framework is particular, hence a
#       filter is added to convert the absolute path to a framework (e.g.
#       /Path/to/Sample.framework) into the correct flags (-F/Path/to/
#       -framework Sample).
#
MACRO(PKG_CONFIG_APPEND_LIBS_RAW LIBS)
  FOREACH(LIB ${LIBS})
    IF(LIB)
      IF( APPLE AND ${LIB} MATCHES "\\.framework")
	    GET_FILENAME_COMPONENT(framework_PATH ${LIB} PATH)
	    GET_FILENAME_COMPONENT(framework_NAME ${LIB} NAME_WE)
        SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} -F${framework_PATH} -Wl,-framework,${framework_NAME}" CACHE INTERNAL "")
      ELSE( APPLE AND ${LIB} MATCHES "\\.framework")
        SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIB}" CACHE INTERNAL "")
      ENDIF( APPLE AND ${LIB} MATCHES "\\.framework")
    ENDIF(LIB)
  ENDFOREACH(LIB ${LIBS})
  STRING(REPLACE "\n" "" _PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS}")
ENDMACRO(PKG_CONFIG_APPEND_LIBS_RAW)

#.rst:
# .. ifmode:: export
#
#   .. command:: PKG_CONFIG_APPEND_LIBS (LIBS)
#
#     This macro adds libraries in a portable way into the pkg-config file.
#
#     Library prefix and suffix is automatically added.
#
#   .. note::
#
#     If you use :variable:`PROJECT_DEBUG_POSTFIX`, this covers both debug
#     and optimized configurations with the correct name for targets
#     affected by the postfix.
#
MACRO(PKG_CONFIG_APPEND_LIBS LIBS)
  FOREACH(LIB ${LIBS})
    IF(LIB)
      # Check if this project is building this library
      IF(TARGET ${LIB})
        SET(LIB_COMPLETE_NAME ${LIB})
        # If OUTPUT_NAME property is defined, use this for the library name.
        GET_PROPERTY(OUTPUT_NAME_SET TARGET ${LIB} PROPERTY OUTPUT_NAME SET)
        IF(OUTPUT_NAME_SET)
          GET_TARGET_PROPERTY(OUTPUT_LIB_NAME ${LIB} OUTPUT_NAME)
        ENDIF(OUTPUT_NAME_SET)
        # If SUFFIX property is defined, use it for defining the library name.
        GET_PROPERTY(SUFFIX_SET TARGET ${LIB} PROPERTY SUFFIX SET)
        IF(SUFFIX_SET)
          GET_TARGET_PROPERTY(LIB_SUFFIX ${LIB} SUFFIX)
        ENDIF(SUFFIX_SET)

        GET_PROPERTY(PREFIX_SET TARGET ${LIB} PROPERTY PREFIX SET)
        IF(PREFIX_SET)
          GET_TARGET_PROPERTY(LIB_PREFIX ${LIB} PREFIX)
        ENDIF(PREFIX_SET)
        IF(OUTPUT_NAME_SET)
          SET(LIB_COMPLETE_NAME ${OUTPUT_LIB_NAME})
        ELSE()
          SET(LIB_COMPLETE_NAME ${LIB_PREFIX}${LIB}${LIB_SUFFIX})
        ENDIF()
        # Remove lib extension if any
        IF(UNIX OR APPLE)
          STRING(REPLACE ".so" "" LIB_COMPLETE_NAME ${LIB_COMPLETE_NAME})
          STRING(REPLACE ".dylib" "" LIB_COMPLETE_NAME ${LIB_COMPLETE_NAME})
        ENDIF(UNIX OR APPLE)
        # Single build type generator
        IF(DEFINED CMAKE_BUILD_TYPE)
          SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBINCL_KW}${LIB_COMPLETE_NAME}${PKGCONFIG_POSTFIX}${LIB_EXT}" CACHE INTERNAL "")
        # Multiple build types generator
        ELSE()
          SET(_PKG_CONFIG_LIBS_DEBUG "${_PKG_CONFIG_LIBS_DEBUG} ${LIBINCL_KW}${LIB_COMPLETE_NAME}${PKGCONFIG_POSTFIX}${LIB_EXT}" CACHE INTERNAL "")
          STRING(STRIP ${_PKG_CONFIG_LIBS_DEBUG} _PKG_CONFIG_LIBS_DEBUG)  # To address CMP0004
          SET(_PKG_CONFIG_LIBS_OPTIMIZED "${_PKG_CONFIG_LIBS_OPTIMIZED} ${LIBINCL_KW}${LIB_COMPLETE_NAME}${LIB_EXT}" CACHE INTERNAL "")
          STRING(STRIP ${_PKG_CONFIG_LIBS_OPTIMIZED} _PKG_CONFIG_LIBS_OPTIMIZED)  # To address CMP0004
        ENDIF()
      ELSE()
        IF(IS_ABSOLUTE ${LIB})
          SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBINCL_ABSKW}${LIB}" CACHE INTERNAL "")
        ELSE()
          SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBINCL_KW}${LIB}${LIB_EXT}" CACHE INTERNAL "")
        ENDIF()
        STRING(STRIP ${_PKG_CONFIG_LIBS} _PKG_CONFIG_LIBS)  # To address CMP0004
      ENDIF()
    ENDIF(LIB)
  ENDFOREACH(LIB ${LIBS})
ENDMACRO(PKG_CONFIG_APPEND_LIBS)


# For internal use only.
# PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# For user look at PKG_CONFIG_USE_COMPILE_DEPENDENCY
#
# This macro changes the target properties to properly search for
# headers  against the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(my-binary my-package)
#
MACRO(PKG_CONFIG_USE_LCOMPILE_DEPENDENCY TARGET PREFIX NO_INCLUDE_SYSTEM)

  IF(DEFINED ${PREFIX}_DEBUG_FOUND)
    FOREACH(FLAG ${${PREFIX}_DEBUG_CFLAGS_OTHER})
      TARGET_COMPILE_OPTIONS(${TARGET} PUBLIC "$<$<CONFIG:Debug>:${FLAG}>")
    ENDFOREACH()
    FOREACH(FLAG ${${PREFIX}_CFLAGS_OTHER})
      TARGET_COMPILE_OPTIONS(${TARGET} PUBLIC "$<$<NOT:$<CONFIG:Debug>>:${FLAG}>")
    ENDFOREACH()
  ELSE()
    FOREACH(FLAG ${${PREFIX}_CFLAGS_OTHER})
      TARGET_COMPILE_OPTIONS(${TARGET} PUBLIC ${FLAG})
    ENDFOREACH()
  ENDIF()

  # Include/libraries paths seems to be filtered on Linux, add paths
  # again.
  SET(SCOPE "PRIVATE")
  IF(PROJECT_USE_KEYWORD_LINK_LIBRARIES)
    SET(SCOPE "PUBLIC")
  ENDIF()
  SET(SYSTEM "SYSTEM")
  IF(NO_INCLUDE_SYSTEM)
    SET(SYSTEM "")
  ENDIF()

  TARGET_INCLUDE_DIRECTORIES(${TARGET} ${SYSTEM} ${SCOPE} ${${PREFIX}_INCLUDE_DIRS})
  IF(DEFINED ${PREFIX}_DEBUG_FOUND)
    TARGET_INCLUDE_DIRECTORIES(${TARGET} ${SYSTEM} ${SCOPE} ${${PREFIX}_DEBUG_INCLUDE_DIRS})
  ENDIF()

ENDMACRO(PKG_CONFIG_USE_LCOMPILE_DEPENDENCY)

MACRO(_FILTER_LINK_FLAGS TARGET IS_GENERAL IS_DEBUG FLAGS)
  FOREACH(FLAG ${FLAGS})
    STRING(FIND "${FLAG}" "/" STARTS_WITH_SLASH)
    STRING(FIND "${FLAG}" "-" STARTS_WITH_DASH)
    IF(NOT WIN32 OR (NOT ${STARTS_WITH_DASH} EQUAL 0 AND NOT ${STARTS_WITH_SLASH} EQUAL 0))
      IF(${IS_GENERAL})
        TARGET_LINK_LIBRARIES(${TARGET} ${PUBLIC_KEYWORD} ${FLAG})
      ELSEIF(${IS_DEBUG})
        TARGET_LINK_LIBRARIES(${TARGET} ${PUBLIC_KEYWORD} debug ${FLAG})
      ELSE()
        TARGET_LINK_LIBRARIES(${TARGET} ${PUBLIC_KEYWORD} optimized ${FLAG})
      ENDIF()
    ENDIF()
  ENDFOREACH()
ENDMACRO()

# Internal use only.
# _PKG_CONFIG_MANIPULATE_LDFLAGS(TARGET PREFIX CONFIG IS_GENERAL IS_DEBUG)
#
MACRO(_PKG_CONFIG_MANIPULATE_LDFLAGS TARGET PREFIX CONFIG IS_GENERAL IS_DEBUG)
  # Make sure we do not override previous flags
  GET_TARGET_PROPERTY(LDFLAGS ${TARGET} LINK_FLAGS${CONFIG})

  # If there were no previous flags, get rid of the XYFLAGS-NOTFOUND
  # in the variables.
  IF(NOT LDFLAGS)
    SET(LDFLAGS "")
  ENDIF()

  # Transform semi-colon seperated list in to space separated list.
  FOREACH(FLAG ${${PREFIX}_LDFLAGS})
    SET(LDFLAGS "${LDFLAGS} ${FLAG}")
  ENDFOREACH()

  # Update the flags.
  SET_TARGET_PROPERTIES(${TARGET}
    PROPERTIES LINK_FLAGS${CONFIG} "${LDFLAGS}")
  _FILTER_LINK_FLAGS(${TARGET} ${IS_GENERAL} ${IS_DEBUG} "${${PREFIX}_LDFLAGS}")
  _FILTER_LINK_FLAGS(${TARGET} ${IS_GENERAL} ${IS_DEBUG} "${${PREFIX}_LDFLAGS_OTHER}")
ENDMACRO(_PKG_CONFIG_MANIPULATE_LDFLAGS TARGET PREFIX CONFIG IS_GENERAL IS_DEBUG)

# Internal use only.
# PKG_CONFIG_USE_LLINK_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# For user look at PKG_CONFIG_USE_LINK_DEPENDENCY
#
# This macro changes the target properties to properly search for
# the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_LLINK_DEPENDENCY(my-binary my-package)
#
MACRO(PKG_CONFIG_USE_LLINK_DEPENDENCY TARGET PREFIX)

  IF(NOT DEFINED ${PREFIX}_DEBUG_FOUND)
    _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} ${PREFIX} "" 1 0)
  ELSE()
    # Single build type generators
    IF(DEFINED CMAKE_BUILD_TYPE)
      STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
      IF("${cmake_build_type}" MATCHES "debug")
        _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} "${PREFIX}_DEBUG" "" 1 0)
      ELSE()
        _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} ${PREFIX} "" 1 0)
      ENDIF()
    # Multiple build types generators
    ELSE()
      FOREACH(config ${CMAKE_CONFIGURATION_TYPES})
        STRING(TOUPPER "_${config}" config_in)
        IF(${config_in} MATCHES "_DEBUG")
          _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} "${PREFIX}_DEBUG" "${config_in}" 0 1)
        ELSE()
          _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} "${PREFIX}" "${config_in}" 0 0)
        ENDIF()
      ENDFOREACH()
    ENDIF()
  ENDIF()

  # Include/libraries paths seems to be filtered on Linux, add paths
  # again.
  LINK_DIRECTORIES(${${PREFIX}_LIBRARY_DIRS})
  IF(DEFINED ${PREFIX}_DEBUG_FOUND)
    LINK_DIRECTORIES(${${PREFIX}_DEBUG_LIBRARY_DIRS})
  ENDIF()

ENDMACRO(PKG_CONFIG_USE_LLINK_DEPENDENCY)

MACRO(BUILD_PREFIX_FOR_PKG DEPENDENCY PREFIX)

  # Transform the dependency into a valid variable prefix.
  # 1. replace invalid characters into underscores.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" LPREFIX "${DEPENDENCY}")
  # 2. make it uppercase.
  STRING(TOUPPER "${LPREFIX}" "LPREFIX")

  # Make sure we search for a previously detected package.
  IF(NOT DEFINED ${LPREFIX}_FOUND)
    MESSAGE(FATAL_ERROR
      "The package ${DEPENDENCY} has not been detected correctly.\n"
      "Have you called ADD_REQUIRED_DEPENDENCY/ADD_OPTIONAL_DEPENDENCY?")
  ENDIF()
  IF(NOT (${LPREFIX}_FOUND OR ${LPREFIX}_DEBUG_FOUND))
    MESSAGE(FATAL_ERROR
      "The package ${DEPENDENCY} has not been found.")
  ENDIF()

  SET(${PREFIX} ${LPREFIX})

ENDMACRO(BUILD_PREFIX_FOR_PKG)

#.rst:
# .. ifmode:: import
#
#   .. command:: PKG_CONFIG_USE_DEPENDENCY (TARGET DEPENDENCY [NO_INCLUDE_SYSTEM])
#
#     This macro changes the target properties to properly search for
#     headers, libraries and link against the required shared libraries
#     when using a dependency detected through pkg-config.
#     I.e.::
#
#       PKG_CONFIG_USE_DEPENDENCY(my-binary my-package)
#
#     :TARGET: Target that will be manipulated by this macro
#
#     :DEPENDENCY: Dependency that will be used
#
#     :NO_INCLUDE_SYSTEM: By default, includes are using the SYSTEM option,
#                         this option changes this behaviour
#

MACRO(PKG_CONFIG_USE_DEPENDENCY TARGET DEPENDENCY)
  SET(options NO_INCLUDE_SYSTEM)
  SET(oneValueArgs )
  SET(multiValueArgs )
  cmake_parse_arguments(PKG_CONFIG_USE_DEPENDENCY "${options}"
    "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)
  PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(${TARGET} ${PREFIX}
    ${PKG_CONFIG_USE_DEPENDENCY_NO_INCLUDE_SYSTEM})
  PKG_CONFIG_USE_LLINK_DEPENDENCY(${TARGET} ${PREFIX})
ENDMACRO(PKG_CONFIG_USE_DEPENDENCY TARGET DEPENDENCY)


#.rst:
# .. ifmode:: import-advanced
#
#   .. command:: PKG_CONFIG_USE_COMPILE_DEPENDENCY (TARGET DEPENDENCY [NO_INCLUDE_SYSTEM])
#
#     This macro changes the target properties to properly search for
#     headers  against the required shared libraries
#     when using a dependency detected through pkg-config.
#
#     :TARGET: Target that will be manipulated by this macro
#
#     :DEPENDENCY: Dependency that will be used
#
#     :NO_INCLUDE_SYSTEM: By default, includes are using the SYSTEM option,
#                         this option changes this behaviour
#
#
MACRO(PKG_CONFIG_USE_COMPILE_DEPENDENCY TARGET DEPENDENCY)
  SET(options NO_INCLUDE_SYSTEM)
  SET(oneValueArgs )
  SET(multiValueArgs )
  cmake_parse_arguments(PKG_CONFIG_USE_DEPENDENCY
    "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)
  PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(${TARGET}
    ${PREFIX}
    ${PKG_CONFIG_USE_COMPILE_DEPENDENCY_NO_INCLUDE_SYSTEM})
ENDMACRO(PKG_CONFIG_USE_COMPILE_DEPENDENCY TARGET DEPENDENCY)

#.rst:
# .. ifmode:: import-advanced
#
#   .. command:: PKG_CONFIG_USE_LINK_DEPENDENCY (TARGET DEPENDENCY)
#
#     This macro changes the target properties to properly search for
#     the required shared libraries
#     when using a dependency detected through pkg-config.
#
MACRO(PKG_CONFIG_USE_LINK_DEPENDENCY TARGET DEPENDENCY)
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)
  PKG_CONFIG_USE_LLINK_DEPENDENCY(${TARGET} ${PREFIX})
ENDMACRO(PKG_CONFIG_USE_LINK_DEPENDENCY TARGET DEPENDENCY)


#.rst:
# .. ifmode:: import-advanced
#
#   .. command:: PKG_CONFIG_ADD_COMPILE_OPTIONS (COMPILE_OPTIONS DEPENDENCY)
#
#     This macro adds the compile-time options for a given pkg-config
#     ``DEPENDENCY`` to a given semi-colon-separated list:
#     ``COMPILE_OPTIONS``. This can be used to provide options to
#     CUDA_ADD_LIBRARY for instance, since it does not support
#     SET_TARGET_PROPERTIES...
#
MACRO(PKG_CONFIG_ADD_COMPILE_OPTIONS COMPILE_OPTIONS DEPENDENCY)
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)

  # If there were no previous options
  IF(NOT ${COMPILE_OPTIONS})
    SET(${COMPILE_OPTIONS} "")
  ENDIF()

  # Append flags
  FOREACH(FLAG ${${PREFIX}_CFLAGS_OTHER})
    LIST(APPEND COMPILE_OPTIONS "${FLAG}")
  ENDFOREACH()
ENDMACRO(PKG_CONFIG_ADD_COMPILE_OPTIONS COMPILE_OPTIONS DEPENDENCY)
