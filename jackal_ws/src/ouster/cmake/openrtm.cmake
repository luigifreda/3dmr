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

# OpenRTM-aist
macro(openrtm)

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_JRL_CMAKE_MODULE_DIR}/find-external/OpenRTM)

  find_package(OpenRTM REQUIRED)
  set(ADDITIONAL_SYMBOL "")
  include_directories(${OPENRTM_INCLUDE_DIRS})
  link_directories(${OPENRTM_LIBRARY_DIRS})
  if(UNIX)
    add_definitions(${OPENRTM_DEFINITIONS})
  endif()

  set(OPENHRP_RTM_IDL_DIR "${OpenHRP_SOURCE_DIR}/idl/OpenRTM/${OPENRTM_VERSION}")
  if( OPENRTM_VERSION STREQUAL OPENRTM_VERSION042 )
    set(ADDITIONAL_SYMBOL "OPENRTM_VERSION_042")
  endif()
  
  if(UNIX)
    set(OPENRTM_IDL_DIR "${OPENRTM_DIR}/include/rtm/idl")
  elseif(WIN32)
    if(OPENRTM_VERSION STREQUAL OPENRTM_VERSION042)
      set(OPENRTM_IDL_DIR "${OPENHRP_RTM_IDL_DIR}")
    else()
      set(OPENRTM_IDL_DIR "${OPENRTM_DIR}/rtm/idl")
    endif()
  endif()
  
  add_definitions(${OMNIORB_CFLAGS})

endmacro(openrtm)

