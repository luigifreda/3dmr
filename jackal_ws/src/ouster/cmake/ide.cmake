# Copyright (C) 2017-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA.
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


FUNCTION(LARGEST_COMMON_PREFIX a b prefix)
  STRING(LENGTH ${a} len_a)
  STRING(LENGTH ${b} len_b)

  IF(${len_a} LESS ${len_b})
      SET(len ${len_a})
  ELSE()
      SET(len ${len_b} )
  ENDIF()

  SET(${prefix} "" PARENT_SCOPE)
  FOREACH(size RANGE 1 ${len})
      STRING(SUBSTRING ${a} 0 ${size} sub_a)
      STRING(SUBSTRING ${b} 0 ${size} sub_b)

      IF(${sub_a} STREQUAL ${sub_b} )
          SET(${prefix} ${sub_a} PARENT_SCOPE)
      ELSE()
          BREAK()
      ENDIF()
  ENDFOREACH()
ENDFUNCTION()

FUNCTION(ADD_GROUP GROUP_NAME FILENAMES)
  SET(REDUCED_FILENAMES)
  FOREACH(filename ${${FILENAMES}})
    GET_FILENAME_COMPONENT(filenamePath ${filename} PATH)
    GET_FILENAME_COMPONENT(filenameName ${filename} NAME)
    STRING(REGEX REPLACE "${PROJECT_BINARY_DIR}" "" filenamePath "${filenamePath}/")
    STRING(REGEX REPLACE "${PROJECT_SOURCE_DIR}" "" filenamePath "${filenamePath}/")
    STRING(REGEX REPLACE "//" "/" filenamePath ${filenamePath})
    LIST(APPEND REDUCED_FILENAMES ${filenamePath})
  ENDFOREACH()

  # Find the largest common prefix
  LIST(LENGTH REDUCED_FILENAMES num_files)
  MATH(EXPR max_id "${num_files}-1")
  IF(${num_files} GREATER 2)
    LIST(GET REDUCED_FILENAMES 0 str_a)
    FOREACH(id RANGE 1 ${max_id})
      LIST(GET REDUCED_FILENAMES ${id} str_b)
      LARGEST_COMMON_PREFIX(${str_a} ${str_b} prefix)
      SET(str_a ${prefix})
      IF("${str_a}" STREQUAL "")
        BREAK()
      ENDIF()
    ENDFOREACH()
  ELSE()
    SET(prefix "")
  ENDIF()

  FOREACH(id RANGE 0 ${max_id})
    LIST(GET ${FILENAMES} ${id} filename)
    LIST(GET REDUCED_FILENAMES ${id} filenamePath)
    IF(NOT ("${prefix}" STREQUAL ""))
      STRING(REGEX REPLACE "${prefix}" "" filenamePath "${filenamePath}")
    ENDIF()
    IF(NOT ("${filenamePath}" STREQUAL ""))
      STRING(REGEX REPLACE "/" "\\\\" filenamePath ${filenamePath})
      SOURCE_GROUP("${GROUP_NAME}\\${filenamePath}" FILES ${filename})
    ELSE()
      SOURCE_GROUP("${GROUP_NAME}" FILES ${filename})
    ENDIF()
  ENDFOREACH()
ENDFUNCTION(ADD_GROUP) 

# ADD_HEADER_GROUP
# ----------------
#
# Add FILENAMES to "Header Files" group when using IDE Cmake Generator
#
MACRO(ADD_HEADER_GROUP FILENAMES)
  ADD_GROUP("Header Files" ${FILENAMES})
ENDMACRO(ADD_HEADER_GROUP FILENAMES)

# ADD_SOURCE_GROUP
# ----------------
#
# Add FILENAMES to "Source Files" group when using IDE Cmake Generator
#
MACRO(ADD_SOURCE_GROUP FILENAMES)
  ADD_GROUP("Source Files" ${FILENAMES})
ENDMACRO(ADD_SOURCE_GROUP FILENAMES)
