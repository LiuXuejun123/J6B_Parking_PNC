# FindOSQP.cmake
# 查找OSQP库的CMake模块

# 设置模块路径，确保CMake能找到这个文件
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# 查找头文件 - 适配不同的目录结构
find_path(OSQP_INCLUDE_DIRS
    NAMES osqp.h
    PATHS
        /usr/include
        /usr/include/osqp
        /usr/local/include
        /usr/local/include/osqp
        ${CMAKE_SOURCE_DIR}/third_party/osqp
        ${CMAKE_SOURCE_DIR}/third_party/osqp/include
    PATH_SUFFIXES
        osqp
)

# 查找库文件 - 适配不同的库名（libosqp.so / osqp.lib 等）
find_library(OSQP_LIBRARIES
    NAMES osqp libosqp osqpstatic
    PATHS
        /usr/lib
        /usr/lib64
        /usr/local/lib
        /usr/local/lib64
        ${CMAKE_SOURCE_DIR}/third_party/osqp
        ${CMAKE_SOURCE_DIR}/third_party/osqp/lib
        ${CMAKE_SOURCE_DIR}/third_party/osqp/build/out
    PATH_SUFFIXES
        Release Debug
)

# 处理标准参数
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OSQP
    FOUND_VAR OSQP_FOUND
    REQUIRED_VARS OSQP_LIBRARIES OSQP_INCLUDE_DIRS
)

# 输出调试信息
if(OSQP_FOUND)
    message(STATUS "✅ OSQP found successfully!")
    message(STATUS "  Include dirs: ${OSQP_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${OSQP_LIBRARIES}")
    
    # 验证头文件是否真的存在
    if(NOT EXISTS "${OSQP_INCLUDE_DIRS}/osqp.h")
        message(WARNING "⚠️ OSQP include dir found but osqp.h not exists in: ${OSQP_INCLUDE_DIRS}")
        set(OSQP_FOUND FALSE)
    endif()
    
    # 验证库文件是否真的存在
    if(NOT EXISTS "${OSQP_LIBRARIES}")
        message(WARNING "⚠️ OSQP library path found but library file not exists: ${OSQP_LIBRARIES}")
        set(OSQP_FOUND FALSE)
    endif()
else()
    message(STATUS "❌ OSQP not found! Search paths:")
    message(STATUS "  Include paths: /usr/include, /usr/local/include, ${CMAKE_SOURCE_DIR}/third_party/osqp/include")
    message(STATUS "  Library paths: /usr/lib, /usr/local/lib, ${CMAKE_SOURCE_DIR}/third_party/osqp/lib")
    message(STATUS "💡 Hint: You can specify OSQP path manually with -DOSQP_INCLUDE_DIRS=/path/to/osqp/include -DOSQP_LIBRARIES=/path/to/libosqp.so")
endif()

mark_as_advanced(OSQP_INCLUDE_DIRS OSQP_LIBRARIES)

