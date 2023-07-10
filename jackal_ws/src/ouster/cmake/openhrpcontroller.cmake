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
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/openrtm.cmake)

macro(create_simple_controller CONTROLLER_NAME)
  openrtm()
  add_library(${CONTROLLER_NAME} SHARED ${CONTROLLER_NAME}.cpp)
  target_link_libraries(${CONTROLLER_NAME} ${OPENRTM_LIBRARIES})
  set_target_properties(${CONTROLLER_NAME} PROPERTIES PREFIX "")

  add_executable(${CONTROLLER_NAME}Comp ${CONTROLLER_NAME}Comp.cpp ${CONTROLLER_NAME}.cpp)
  target_link_libraries(${CONTROLLER_NAME}Comp ${OPENRTM_LIBRARIES})

  if(WIN32)
    add_definitions(${OPENRTM_DEFINITIONS})
    set_target_properties(${CONTROLLER_NAME}Comp PROPERTIES DEBUG_POSTFIX d )
  endif()
  
  set(controller_install_path lib/openhrp/controller/${CONTROLLER_NAME})	
 
  install(TARGETS ${CONTROLLER_NAME} ${CONTROLLER_NAME}Comp DESTINATION ${controller_install_path} CONFIGURATIONS Release)
  
  if(WIN32)
    install(TARGETS ${CONTROLLER_NAME} ${CONTROLLER_NAME}Comp
        DESTINATION ${PROJECT_SOURCE_DIR}
        CONFIGURATIONS Release )
  endif()

  install(FILES rtc.conf bridge.conf DESTINATION ${controller_install_path})

  if(EXISTS ${PROJECT_SOURCE_DIR}/etc)
    install(DIRECTORY etc DESTINATION ${controller_install_path} PATTERN ".svn" EXCLUDE)
  endif()

endmacro()

