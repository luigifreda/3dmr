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

# SPHINX_SETUP()
# --------------
#
# Look for Sphinx, add a custom rule to generate the documentation and
# install the documentation properly.
#
MACRO(SPHINX_SETUP)
  SET(SPHINX_BUILD_PATH "")

  #  With MSVC, it is likely thant sphinx has been installed using easy-install
  # directly in the python folder.
  IF (MSVC)
    GET_FILENAME_COMPONENT(PYTHON_ROOT ${PYTHON_EXECUTABLE} DIRECTORY)
    SET(SPHINX_BUILD_PATH ${PYTHON_ROOT}/Scripts)
  ENDIF(MSVC)

  FIND_PROGRAM(SPHINX_BUILD sphinx-build DOC
    "Sphinx documentation generator tool"
    PATHS "${SPHINX_BUILD_PATH}")

  IF (NOT SPHINX_BUILD)
    MESSAGE(WARNING "Failed to find sphinx, documentation will not be generated.")
  ELSE (NOT SPHINX_BUILD)

    IF(MSVC)
      # FIXME: it is impossible to trigger documentation installation
      # at install, so put the target in ALL instead.
      ADD_CUSTOM_TARGET(sphinx-doc ALL
        COMMAND ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b html ${CMAKE_CURRENT_BINARY_DIR}/sphinx
        ${CMAKE_CURRENT_BINARY_DIR}/sphinx-html
        COMMENT "Generating sphinx documentation"
        )
    ELSEIF(APPLE)
      # THE DYLD_LIBRARY_PATH should be completed to run the sphinx command.
      #  otherwise some symbols won't be found.
      SET(EXTRA_LD_PATH "\"${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\":")
      SET(EXTRA_LD_PATH "${EXTRA_LD_PATH}\"${DYNAMIC_GRAPH_PLUGINDIR}\":")
      ADD_CUSTOM_TARGET(sphinx-doc
        COMMAND  export DYLD_LIBRARY_PATH=${EXTRA_LD_PATH}:\$DYLD_LIBRARY_PATH \;
        ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b html ${CMAKE_CURRENT_BINARY_DIR}/sphinx
        ${CMAKE_CURRENT_BINARY_DIR}/sphinx-html
        COMMENT "Generating sphinx documentation"
        )

      IF(INSTALL_DOCUMENTATION)
        INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} sphinx-doc)")
      ENDIF(INSTALL_DOCUMENTATION)
    ELSE() #UNIX
      # THE LD_LIBRARY_PATH should be completed to run the sphinx command.
      #  otherwise some symbols won't be found.
      SET(EXTRA_LD_PATH "\"${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\":")
      SET(EXTRA_LD_PATH "${EXTRA_LD_PATH}\"${DYNAMIC_GRAPH_PLUGINDIR}\":")
      ADD_CUSTOM_TARGET(sphinx-doc
        COMMAND  export LD_LIBRARY_PATH=${EXTRA_LD_PATH}:$$LD_LIBRARY_PATH \;
        ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b html ${CMAKE_CURRENT_BINARY_DIR}/sphinx
        ${CMAKE_CURRENT_BINARY_DIR}/sphinx-html
        COMMENT "Generating sphinx documentation"
        )

      IF(INSTALL_DOCUMENTATION)
        INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} sphinx-doc)")
      ENDIF(INSTALL_DOCUMENTATION)
    ENDIF(MSVC)

    ADD_CUSTOM_COMMAND(
      OUTPUT
      ${CMAKE_BINARY_DIR}/doc/sphinx-html
      COMMAND ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b html  ${CMAKE_CURRENT_BINARY_DIR}/sphinx
        ${CMAKE_CURRENT_BINARY_DIR}/sphinx-html
      COMMENT "Generating sphinx documentation"
      )

    # Clean generated files.
    SET_PROPERTY(
      DIRECTORY APPEND PROPERTY
      ADDITIONAL_MAKE_CLEAN_FILES
      ${CMAKE_BINARY_DIR}/doc/sphinx-html
      )

    # Install generated files.
    IF(INSTALL_DOCUMENTATION)
      INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/doc/sphinx-html
        DESTINATION share/doc/${PROJECT_NAME})

      IF(EXISTS ${PROJECT_SOURCE_DIR}/doc/pictures)
        INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/doc/pictures
          DESTINATION share/doc/${PROJECT_NAME}/sphinx-html)
      ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/doc/pictures)
    ENDIF(INSTALL_DOCUMENTATION)

  ENDIF(NOT SPHINX_BUILD)

  LIST(APPEND LOGGING_WATCHED_VARIABLES
    SPHINX_BUILD
    )
ENDMACRO(SPHINX_SETUP)


# SPHINX_FINALIZE()
# -----------------
#
# Generate Sphinx related files.
#
MACRO(SPHINX_FINALIZE)
  IF (SPHINX_BUILD)
  CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/sphinx/index.rst.in
    ${CMAKE_CURRENT_BINARY_DIR}/sphinx/index.rst
    @ONLY
    )

  CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/sphinx/conf.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/sphinx/conf.py
    @ONLY
    )
  ENDIF(SPHINX_BUILD)
ENDMACRO(SPHINX_FINALIZE)
