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

INCLUDE(CheckCXXCompilerFlag)

MACRO(_SETUP_PROJECT_WARNINGS)
  # -Wmissing-declarations is disabled for now
  # as older GCC version does not support it
  # but CMake doest not check for the flag acceptance
  # correctly.

 IF(UNIX)
  SET(FLAGS -pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual
            -Wformat -Wwrite-strings -Wconversion)
  FOREACH(FLAG ${FLAGS})
    CHECK_CXX_COMPILER_FLAG(${FLAG} R${FLAG})
    IF(${R${FLAG}})
      SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
    ENDIF(${R${FLAG}})
  ENDFOREACH(FLAG ${FLAGS})

  IF(NOT DEFINED CXX_DISABLE_WERROR)
    SET(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
  ENDIF(NOT DEFINED CXX_DISABLE_WERROR)
 ENDIF(UNIX)

 # For win32 systems, it is impossible to use Wall,
 # especially with boost, which is way too verbose
 # The default levels (W3/W4) are enough
 # The next macro remove warnings on deprecations due to stl.
 IF(MSVC)
  SET(WARNING_CXX_FLAGS "-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS")
  SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} -D_CRT_SECURE_NO_DEPRECATE")
  ## -- The following warnings are removed to highlight the output
  # C4101 The local variable is never used
  # removed since happens frequently in headers.
  SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} /wd4101")
  # C4250 'class1' : inherits 'class2::member' via dominance
  SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} /wd4250")
  # C4251 class 'type' needs to have dll-interface to be used by clients of class 'type2'
  # ~ in practice, raised by the classes that have non-dll attribute (such as std::vector)
  SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} /wd4251")
  # C4275 non - DLL-interface used as base for DLL-interface
  SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} /wd4275")
  # C4355 "this" used in base member initializer list
  SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} /wd4355")
 ENDIF()

 SET(CMAKE_CXX_FLAGS "${WARNING_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

 LIST(APPEND LOGGING_WATCHED_VARIABLES WARNING_CXX_FLAGS)
ENDMACRO(_SETUP_PROJECT_WARNINGS)
