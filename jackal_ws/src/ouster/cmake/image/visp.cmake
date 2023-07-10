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

# FIND_VISP()
# -----------
#
# Find ViSP through the use of visp-config.
#
# To make ViSP detection both robust and simple, rely on the
# visp-config tool instead of the specic CMake mechanism.
#
# One just has to make sure that visp-config is available in its path
# to make ViSP detected automagically.
#
# Implementation note: please do *not* modify this code to add a
# variable which override the path and the standard look-up algorithm,
# it introduces complexity for nothing. Fix your path instead.
MACRO(FIND_VISP)
  FIND_PROGRAM(VISP_CONFIG visp-config)
  IF(NOT VISP_CONFIG)
    MESSAGE(FATAL_ERROR "visp-config has not been found.")
  ENDIF()
  EXECUTE_PROCESS(
    COMMAND ${VISP_CONFIG} --prefix
    OUTPUT_VARIABLE VISP_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  EXECUTE_PROCESS(
    COMMAND ${VISP_CONFIG} --cflags
    OUTPUT_VARIABLE VISP_CFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  EXECUTE_PROCESS(
    COMMAND ${VISP_CONFIG} --libs
    OUTPUT_VARIABLE VISP_LIBS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  EXECUTE_PROCESS(
    COMMAND ${VISP_CONFIG} --dumpversion
    OUTPUT_VARIABLE VISP_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDMACRO(FIND_VISP)
