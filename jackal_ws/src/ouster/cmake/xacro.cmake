# Copyright (C) 2016 LAAS-CNRS, JRL AIST-CNRS.
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

#.rst:
#
# .. variable:: XACRO_OPTIONS
#
#    Options passed to the xacro command. It defaults to ``--inorder``.
#
# .. command:: RUN_XACRO (INPUT OUTPUT)
#
#   Add a custom command that runs the following command:
#
#     xacro.py -o ${OUTPUT} ${INPUT}
#
#   To trigger generation, use::
#
#     ADD_CUSTOM_TARGET (generate_urdf_files ALL DEPENDS ${ALL_GENERATED_URDF})
#
#   See also :cmake:variable:`XACRO_OPTIONS`.
#
MACRO(RUN_XACRO INPUT OUTPUT)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS xacro)

  IF(NOT DEFINED XACRO_OPTIONS)
    SET(XACRO_OPTIONS "--inorder")
  ENDIF()

  ADD_CUSTOM_COMMAND(
    OUTPUT ${OUTPUT}
    COMMAND ${_xacro_py}
    ARGS ${XACRO_OPTIONS} -o ${OUTPUT} ${INPUT}
    MAIN_DEPENDENCY ${INPUT}
    COMMENT "Generating ${OUTPUT}"
    )
  LIST(APPEND ALL_GENERATED_URDF ${OUTPUT})

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${OUTPUT}
    )

  LIST(APPEND LOGGING_WATCHED_VARIABLES ALL_GENERATED_URDF)
ENDMACRO(RUN_XACRO INPUT OUTPUT)

#.rst:
# .. command:: GENERATE_URDF_FILE (FILENAME EXTENSION)
#
#   Generate urdf ``${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}.${EXTENSION}``
#   from xacro ``${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro`` file.
#
#   To trigger generation, use::
#
#     ADD_CUSTOM_TARGET (generate_urdf_files DEPENDS ${ALL_GENERATED_URDF})
#
#   :FILENAME:  XACRO filename without the extension
#   :EXTENSION: desired extension of the output file, e.g. "urdf" or "srdf"
#
#   .. note:: If ``${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro`` does not exists,
#          the macros tries to configure file ``${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in``
#
MACRO(GENERATE_URDF_FILE FILENAME EXTENSION)
  IF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)
    IF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in)
      MESSAGE(FATAL_ERROR "cannot find \"${FILENAME}.xacro\" or \"${FILENAME}.xacro.in\"")
    ENDIF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in)

    SET(_XACRO_FILE_ ${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}.xacro)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in
      ${_XACRO_FILE_} @ONLY)
    # MESSAGE("Configuring ${FILENAME}.xacro.in")
  ELSE (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)
    SET(_XACRO_FILE_ ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)
  ENDIF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)

  RUN_XACRO(${_XACRO_FILE_} ${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_FILE})
ENDMACRO(GENERATE_URDF_FILE FILENAME CONFIGURE)
