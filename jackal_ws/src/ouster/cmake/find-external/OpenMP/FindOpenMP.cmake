find_library(OpenMP_CXX_LIBRARY
    NAMES omp
)

find_path(OpenMP_CXX_INCLUDE_DIR
    omp.h
)

mark_as_advanced(OpenMP_CXX_LIBRARY OpenMP_CXX_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenMP DEFAULT_MSG 
    OpenMP_CXX_LIBRARY OpenMP_CXX_INCLUDE_DIR)

if(OpenMP_FOUND)
    set(OpenMP_CXX_LIBRARIES ${OpenMP_CXX_LIBRARY})
    set(OpenMP_CXX_INCLUDE_DIRS ${OpenMP_CXX_INCLUDE_DIR})
    set(OpenMP_CXX_COMPILE_OPTIONS -Xpreprocessor -fopenmp)
    set(OpenMP_CXX_FLAGS ${OpenMP_CXX_COMPILE_OPTIONS})

    add_library(OpenMP::OpenMP_CXX SHARED IMPORTED)
    set_target_properties(OpenMP::OpenMP_CXX PROPERTIES
        IMPORTED_LOCATION ${OpenMP_CXX_LIBRARIES}
        INTERFACE_INCLUDE_DIRECTORIES "${OpenMP_CXX_INCLUDE_DIRS}"
        INTERFACE_COMPILE_OPTIONS "${OpenMP_CXX_COMPILE_OPTIONS}"
    )
endif()

