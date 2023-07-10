
# @author Shin'ichiro Nakaoka

set(OPENRTM_FOUND FALSE)
set(OPENRTM_VERSION042 "0.4.2")
set(OPENRTM_VERSION100 "1.0.0")
set(OPENRTM_PKG_CONFIG_FOUND FALSE)
#Additional threshold OpenRTM versions switching processing 
set(OPENRTM_DEFAULT_VERSION ${OPENRTM_VERSION100})

if(UNIX)
  if(OPENRTM_DIR)
    string(REGEX REPLACE "/$" "" OPENRTM_DIR ${OPENRTM_DIR})
    set(OPENRTM_CONFIG_EXECUTABLE ${OPENRTM_DIR}/bin/rtm-config)
    if(NOT EXISTS ${OPENRTM_CONFIG_EXECUTABLE})
      set(OPENRTM_CONFIG_EXECUTABLE)
      message(FATAL_ERROR "rtm-config was not found in ${OPENRTM_DIR}/bin. Please set OPENRTM_DIR correctly.")
    endif()
  else()
    find_program(OPENRTM_CONFIG_EXECUTABLE rtm-config DOC "The location of the rtm-config script")
    mark_as_advanced(OPENRTM_CONFIG_EXECUTABLE)  
  endif()

  if(OPENRTM_CONFIG_EXECUTABLE)
    set(OPENRTM_FOUND TRUE)

    execute_process(
      COMMAND ${OPENRTM_CONFIG_EXECUTABLE} --version
      OUTPUT_VARIABLE OPENRTM_VERSION
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT RESULT EQUAL 0)
      set(OPENRTM_FOUND FALSE)
    endif()

    execute_process(
      COMMAND ${OPENRTM_CONFIG_EXECUTABLE} --prefix
      OUTPUT_VARIABLE OPENRTM_DIR
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(RESULT EQUAL 0)
      if(OPENRTM_DIR)
        list(APPEND OPENRTM_INCLUDE_DIRS "${OPENRTM_DIR}/include")
        list(APPEND OPENRTM_INCLUDE_DIRS "${OPENRTM_DIR}/include/rtm/idl")
      endif()
    else()
      set(OPENRTM_FOUND FALSE)
    endif()

    execute_process(
      COMMAND ${OPENRTM_CONFIG_EXECUTABLE} --cflags
      OUTPUT_VARIABLE OPENRTM_CXX_FLAGS
      RESULT_VARIABLE RESULT)

    if(RESULT EQUAL 0)
      string(REGEX MATCHALL "-D.*[^ ;]+" OPENRTM_DEFINITIONS ${OPENRTM_CXX_FLAGS})
    else()
      set(OPENRTM_FOUND FALSE)
    endif()

    execute_process(
      COMMAND ${OPENRTM_CONFIG_EXECUTABLE} --libs
      OUTPUT_VARIABLE OPENRTM_LIBRARIES
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(RESULT EQUAL 0)
      string(REGEX MATCHALL "-L[^ ;]+" OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARIES})
      string(REGEX REPLACE "-L" ";" OPENRTM_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS})
      string(REGEX REPLACE "-L[^ ;]+" "" OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES})
      separate_arguments(OPENRTM_LIBRARIES)
    else()
      set(OPENRTM_FOUND FALSE)
    endif()

  endif(OPENRTM_CONFIG_EXECUTABLE)
  set(OPENRTM_PKG_CONFIG_FILE "/usr/lib/pkgconfig/openrtm-aist.pc")
  if(EXISTS ${OPENRTM_PKG_CONFIG_FILE})
    set(OPENRTM_PKG_CONFIG_FOUND TRUE)
  endif()
endif(UNIX)

if(WIN32)
  set(OPENRTM_DEFINITIONS -DUSE_stub_in_nt_dll )
  if( NOT OPENRTM_DIR )
    if(NOT $ENV{RTM_ROOT} STREQUAL "")
      set(OPENRTM_DIR $ENV{RTM_ROOT})
    endif()
  endif()
  if(OPENRTM_DIR )
    if( ${OPENRTM_DIR} MATCHES ".*\\\\0\\.4\\\\$" )
      set(OPENRTM_DEFAULT_VERSION ${OPENRTM_VERSION042})
    elseif(${OPENRTM_DIR} MATCHES ".*\\/0\\.4$")
      set(OPENRTM_DEFAULT_VERSION ${OPENRTM_VERSION042})
    else()
      set(OPENRTM_DEFAULT_VERSION ${OPENRTM_VERSION100})
    endif()
    set(OPENRTM_VERSION ${OPENRTM_DEFAULT_VERSION} CACHE STRING "Set version of OpenRTM-aist. Default version is ${OPENRTM_DEFAULT_VERSION}")
    set(OPENRTM_INCLUDE_DIRS ${OPENRTM_DIR} )
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/bin )
    list(APPEND OPENRTM_INCLUDE_DIRS "${OPENRTM_DIR}/rtm/idl")
    if(OPENRTM_VERSION STREQUAL ${OPENRTM_VERSION042})
      set(OPENRTM_LIBRARIES_RELEASE RTC042 ACE )
    else()
      # 1.0.0-RC
      #set(OPENRTM_LIBRARIES_RELEASE RTC100 coil)

      # 1.0.0-Release
      set(OPENRTM_LIBRARIES_RELEASE RTC100 coil)
      set(OPENRTM_NODEBUG_LIBRARIES ws2_32 mswsock)
    endif()
    foreach(library ${OPENRTM_LIBRARIES_RELEASE})
      list(APPEND OPENRTM_LIBRARIES optimized ${library} debug ${library}d )
    endforeach()
    foreach(library ${OPENRTM_NODEBUG_LIBRARIES})
      list(APPEND OPENRTM_LIBRARIES optimized ${library} debug ${library} )
    endforeach()
  endif()

  if("${OPENRTM_VERSION}" STREQUAL ${OPENRTM_VERSION042})
    if(NOT ACE_ROOT)
      if(NOT $ENV{ACE_ROOT} STREQUAL "")
        set(ACE_ROOT $ENV{ACE_ROOT})
      endif()
      set(ACE_ROOT ${ACE_ROOT} CACHE PATH "The top directory of ACE")
    endif()
    if(ACE_ROOT)
      include_directories(${ACE_ROOT})
      link_directories(${ACE_ROOT}/lib)
    endif()
    if(OPENRTM_DIR AND ACE_ROOT)
      set(OPENRTM_FOUND TRUE)
    endif()
  else()
    if(OPENRTM_DIR)
       set(OPENRTM_FOUND TRUE)
    endif()
  endif()
endif(WIN32)

if(NOT OPENRTM_FOUND)
  set(OPENRTM_DIR NOT_FOUND)
endif()

set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "The top directory of OpenRTM-aist")

if(OPENRTM_FOUND)
  if( ${OPENRTM_VERSION} MATCHES "^0\\." )
      message(FATAL_ERROR "Not support OpenRTM-aist Ver.${OPENRTM_VERSION}, please install OpenRTM-aist Ver.1.0.0 or later and specify it's location.")
  endif()

  if(NOT OpenRTM_FIND_QUIETLY)
    message(STATUS "Found OpenRTM-aist ${OPENRTM_VERSION} in ${OPENRTM_DIR}")
  endif()
  if(OPENRTM_PKG_CONFIG_FOUND)
    message(STATUS "OpenRTM is supported pkg-config.")
  else()
    message(STATUS "OpenRTM is not supported pkg-config.")
    if(WIN32)
      list(APPEND OPENRTM_LIBRARIES ${OMNIORB_LIBRARIES})
    endif(WIN32)
  endif()
else()
  if(NOT OpenRTM_FIND_QUIETLY)
    if(OpenRTM_FIND_REQUIRED)
      message(FATAL_ERROR "OpenRTM-aist required, please specify it's location.")
    endif()
  endif()
endif()
