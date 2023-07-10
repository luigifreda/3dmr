# Copyright (C) 2018-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA
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

include(CheckCXXCompilerFlag)

#.rst:
# .. ifmode:: user
#
# .. command:: CHECK_CXX11_SUPPORT
#
#    Set ouput variable CXX11_SUPPORTED to TRUE if C++11 is supported
#    by the current compiler. Set to FALSE otherwise.
#
FUNCTION(CHECK_CXX11_SUPPORT CXX11_SUPPORTED)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

  IF(COMPILER_SUPPORTS_CXX0X OR COMPILER_SUPPORTS_CXX11)
    SET(${CXX11_SUPPORTED} TRUE PARENT_SCOPE)
  ELSE()
    SET(${CXX11_SUPPORTED} FALSE PARENT_SCOPE)
  ENDIF()
ENDFUNCTION(CHECK_CXX11_SUPPORT)

#.rst:
# .. ifmode:: user
#
# .. command:: PROJECT_USE_CXX11
#
#    This macro set up the project to compile the whole project 
#    with C++11 standards.  
#
MACRO(PROJECT_USE_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  IF(COMPILER_SUPPORTS_CXX0X OR COMPILER_SUPPORTS_CXX11)
    IF(CMAKE_VERSION VERSION_LESS "3.1")
      IF(COMPILER_SUPPORTS_CXX0X)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
      ELSEIF(COMPILER_SUPPORTS_CXX11)  
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
      ENDIF()
    ELSE()
      SET(CMAKE_CXX_STANDARD 11)
      SET(CXX_STANDARD_REQUIRED ON)
    ENDIF()
  ELSE()
    MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
  ENDIF()
ENDMACRO(PROJECT_USE_CXX11)
