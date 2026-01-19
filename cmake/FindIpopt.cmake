# FindIpopt.cmake
# 查找Ipopt库的CMake模块

find_path(Ipopt_INCLUDE_DIR
    NAMES IpoptConfig.h IpNLP.hpp
    PATHS
        /usr/include/coin
        /usr/local/include/coin
        ${CMAKE_SOURCE_DIR}/third_party/ipopt/include
)

find_library(Ipopt_LIBRARIES
    NAMES ipopt
    PATHS
        /usr/lib
        /usr/local/lib
        ${CMAKE_SOURCE_DIR}/third_party/ipopt/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ipopt
    FOUND_VAR Ipopt_FOUND
    REQUIRED_VARS Ipopt_LIBRARIES Ipopt_INCLUDE_DIR
)

if(Ipopt_FOUND)
    message(STATUS "Ipopt found:")
    message(STATUS "  Include dir: ${Ipopt_INCLUDE_DIR}")
    message(STATUS "  Libraries: ${Ipopt_LIBRARIES}")
endif()

mark_as_advanced(Ipopt_INCLUDE_DIR Ipopt_LIBRARIES)
