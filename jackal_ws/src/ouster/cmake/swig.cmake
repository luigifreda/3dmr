MACRO(GENERATE_INCLUDE_FLAGS)
	get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
	SET(INCLUDE_FLAGS "")
	foreach(dir ${dirs})
		SET(INCLUDE_FLAGS "-I${dir}" ${INCLUDE_FLAGS})
	endforeach()
ENDMACRO(GENERATE_INCLUDE_FLAGS)

MACRO(ADD_SWIG_PYTHON_BINDING FILENAME DIRECTORY)
	FIND_PROGRAM(SWIG swig)
	IF(${SWIG} STREQUAL SWIG-NOTFOUND)
		MESSAGE(FATAL_ERROR "Cannot find swig")
	ENDIF()
	GENERATE_INCLUDE_FLAGS()
	SET(outname ${CMAKE_BINARY_DIR}/${DIRECTORY}/${FILENAME}_wrap.cxx)
	ADD_CUSTOM_COMMAND(
		OUTPUT ${outname}
		COMMAND ${SWIG}
		ARGS -c++ -python -outcurrentdir ${INCLUDE_FLAGS} ${CMAKE_SOURCE_DIR}/${DIRECTORY}/${FILENAME}.i
		MAIN_DEPENDENCY ${FILENAME}.i
		)
	SET(PYTHON_SWIG_SOURCES ${FILENAME} ${PYTHON_SWIG_SOURCES})
	SET(PYTHON_SWIG_STUBS ${CMAKE_BINARY_DIR}/${DIRECTORY}/${FILENAME}_wrap.cxx ${PYTHON_SWIG_STUBS})
ENDMACRO(ADD_SWIG_PYTHON_BINDING FILENAME)

MACRO(GENERATE_SWIG_BINDINGS)
	ADD_CUSTOM_TARGET(generate_python_bindings DEPENDS ${PYTHON_SWIG_STUBS})
ENDMACRO(GENERATE_SWIG_BINDINGS)

MACRO(BUILD_SWIG_BINDINGS LIBRARIES)
	foreach(stub ${PYTHON_SWIG_SOURCES})
		set(libname "${stub}_lib")
		set(realname "_${stub}")
		set(stubname "${stub}_wrap.cxx")
		set(stubpath "${CMAKE_BINARY_DIR}/binding/${stubname}")
		set(SWIG_TARGETS ${libname} ${SWIG_TARGETS})
		set_source_files_properties(${stubpath} PROPERTIES GENERATED 1)
		add_library(${libname} SHARED ${stubpath})
		target_link_libraries(${libname} ${LIBRARIES})
		add_dependencies(${libname} generate_python_bindings)
		set_target_properties(${libname} PROPERTIES OUTPUT_NAME ${realname}
						 PREFIX "")
	endforeach()
ENDMACRO()

MACRO(INSTALL_SWIG_BINDINGS PYTHON_SITELIB PACKAGE)
	foreach(target ${SWIG_TARGETS})
		INSTALL(TARGETS ${target} DESTINATION ${PYTHON_SITELIB}/${PACKAGE})
	endforeach()
	foreach(source ${PYTHON_SWIG_SOURCES})
		INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/binding/${source}.py
			DESTINATION ${PYTHON_SITELIB}/${PACKAGE})
	endforeach()
ENDMACRO()
