# add_doxygen() This function will find Doxygen and and create a documentation target
# 
# There are two keyword arguments
#
# REQUIRED -- This will cause a failure if Doxygen is not found by cmake
#             If this is not added, failing to find Doxygen will simply
#             not build any documentation.
# NOT_AUTOMATIC -- If this option is not added, the documentation is
#                  built when calling make. If this option is added
#                  you must build the documentation manually by calling
#                  make doc
function(add_doxygen)
  cmake_parse_arguments(add_dox_args "REQUIRED;NOT_AUTOMATIC" "" "" ${ARGN})

  if(add_dox_args_REQUIRED)
    find_package(Doxygen REQUIRED)
  else()
    find_package(Doxygen)
  endif()

  set(DOXYFILE "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")
  if(EXISTS ${DOXYFILE})
  else()
    message("The function add_doxygen() expects that the file, 
Doxyfile.in exists in your package directory,

${CMAKE_CURRENT_SOURCE_DIR}

You can generate a default configuration file with 

doxygen [-s] -g Doxyfile.in
")
  endif()

  set(doxygen_catkin_DOC_DIR "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/doc")
  file(MAKE_DIRECTORY ${doxygen_catkin_DOC_DIR})

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

  
  if(DOXYGEN_FOUND)
    # Stop the automatic inclusion of the doc target
    if(add_dox_args_NOT_AUTOMATIC)
      # add a target to generate API documentation with Doxygen
      add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${doxygen_catkin_DOC_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
    else()
      # add a target to generate API documentation with Doxygen
      add_custom_target(doc ALL
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${doxygen_catkin_DOC_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
    endif()

    install(DIRECTORY ${doxygen_catkin_DOC_DIR} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  else()
    message("Unable to find Doxygen. API Documentation will not be generated")
  endif()

endfunction()