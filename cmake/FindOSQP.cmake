# FindOSQP.cmake
# 查找OSQP库的CMake模块

find_path(OSQP_INCLUDE_DIRS
    NAMES osqp.h osqp/include/osqp.h
    PATHS
        /usr/include
        /usr/local/include
        ${CMAKE_SOURCE_DIR}/third_party/osqp/include
)

find_library(OSQP_LIBRARIES
    NAMES osqp
    PATHS
        /usr/lib
        /usr/local/lib
        ${CMAKE_SOURCE_DIR}/third_party/osqp/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OSQP
    FOUND_VAR OSQP_FOUND
    REQUIRED_VARS OSQP_LIBRARIES OSQP_INCLUDE_DIRS
)

if(OSQP_FOUND)
    message(STATUS "OSQP found:")
    message(STATUS "  Include dirs: ${OSQP_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${OSQP_LIBRARIES}")
endif()

mark_as_advanced(OSQP_INCLUDE_DIRS OSQP_LIBRARIES)
