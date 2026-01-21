# cmake_minimum_required(VERSION 3.12)
# project(J6B_Parking_PNC VERSION 1.0.0 LANGUAGES CXX)

# # 设置C++标准
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# # 设置编译选项
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# # 设置输出目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# # 添加CMake模块路径
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# # 查找依赖库
# find_package(PkgConfig REQUIRED)

# # 查找yaml-cpp
# find_package(yaml-cpp REQUIRED)
# if(NOT yaml-cpp_FOUND)
#     message(FATAL_ERROR "yaml-cpp not found. Please install: sudo apt-get install libyaml-cpp-dev")
# endif()

# # 查找Eigen3 (可选)
# find_package(Eigen3 QUIET)
# if(Eigen3_FOUND)
#     message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
#     add_definitions(-DUSE_EIGEN)
#     include_directories(${EIGEN3_INCLUDE_DIR})
# endif()

# # 查找OSQP (用于轨迹平滑)
# find_package(OSQP QUIET)
# if(OSQP_FOUND)
#     message(STATUS "OSQP found")
#     add_definitions(-DUSE_OSQP)
# else()
#     message(WARNING "OSQP not found. Trajectory smoothing will be disabled.")
# endif()

# # 查找Ipopt (用于轨迹优化)
# find_package(Ipopt QUIET)
# if(Ipopt_FOUND)
#     message(STATUS "Ipopt found: ${Ipopt_INCLUDE_DIR}")
#     add_definitions(-DUSE_IPOPT)
# else()
#     message(WARNING "Ipopt not found. Trajectory optimization will be disabled.")
# endif()

# # 包含目录
# include_directories(
#     ${CMAKE_SOURCE_DIR}/src
#     ${CMAKE_SOURCE_DIR}/include
# )

# # ============================================
# # 构建各个模块
# # ============================================

# # Common模块 (配置读取器和公共数据类型)
# add_subdirectory(src/common)

# # GridMap模块 (栅格地图构建)
# add_subdirectory(src/grid_map)

# # ParkingSpaceEvaluation模块 (车位评估)
# add_subdirectory(src/parking_space_evaluation)

# # ParkingSpaceRecommendation模块 (车位推荐)
# add_subdirectory(src/parking_sapce_recommendation)

# # HybridAstar模块 (混合A*路径规划)
# add_subdirectory(src/hybrid_astar)

# # TrajectorySmoothing模块 (轨迹平滑)
# add_subdirectory(src/trajectory_smoothing)

# # VelocityPlanning模块 (速度规划)
# add_subdirectory(src/velocity_planning)

# # TrajectoryMerging模块 (轨迹合并)
# add_subdirectory(src/trajectory_merging)

# # TrajectoryOptimization模块 (轨迹优化)
# add_subdirectory(src/trajectory_optimization)

# # ControlTrajectoryOutput模块 (控制轨迹输出)
# add_subdirectory(src/trajectory_output)

# # Core模块 (统一接口类，封装所有模块)
# add_subdirectory(src/core)

# # ============================================
# # 创建聚合库 (可选，方便一次性链接所有模块)
# # 直接使用核心接口类的实现作为源文件
# # ============================================
# add_library(j6b_parking_pnc_all STATIC
#     src/core/J6BParkingPlanner.cpp
# )

# target_link_libraries(j6b_parking_pnc_all PUBLIC
#     core  # 核心接口类，已经链接了所有其他模块
# )

# # 设置聚合库的输出名称
# set_target_properties(j6b_parking_pnc_all PROPERTIES
#     OUTPUT_NAME j6b_parking_pnc
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
# )

# # ============================================
# # 安装规则
# # ============================================
# install(TARGETS 
#     common
#     grid_map
#     parking_space_evaluation
#     parking_space_recommendation
#     hybrid_astar
#     trajectory_smoothing
#     velocity_planning
#     trajectory_merging
#     trajectory_optimization
#     trajectory_output
#     core
#     j6b_parking_pnc_all
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
# )

# # ============================================
# # 测试支持
# # ============================================
# enable_testing()
# if(EXISTS ${CMAKE_SOURCE_DIR}/test/CMakeLists.txt)
#     add_subdirectory(test)
# endif()

# # ============================================
# # 示例程序支持
# # ============================================
# if(EXISTS ${CMAKE_SOURCE_DIR}/example/CMakeLists.txt)
#     add_subdirectory(example)
# endif()
cmake_minimum_required(VERSION 3.12)
project(J6B_Parking_PNC VERSION 1.0.0 LANGUAGES CXX)

# ===================== 基础配置 =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# 设置输出目录（静态库到build/lib，头文件到build/include）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# 定义头文件输出目录（build/include）
set(HEADER_OUTPUT_DIR ${CMAKE_BINARY_DIR}/include)

# 添加CMake模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# ===================== 依赖查找 =====================
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT yaml-cpp_FOUND)
    message(FATAL_ERROR "yaml-cpp not found. Please install: sudo apt-get install libyaml-cpp-dev")
endif()

find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
    add_definitions(-DUSE_EIGEN)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(OSQP QUIET)
if(OSQP_FOUND)
    message(STATUS "OSQP found")
    add_definitions(-DUSE_OSQP)
else()
    message(WARNING "OSQP not found. Trajectory smoothing will be disabled.")
endif()

find_package(Ipopt QUIET)
if(Ipopt_FOUND)
    message(STATUS "Ipopt found: ${Ipopt_INCLUDE_DIR}")
    add_definitions(-DUSE_IPOPT)
else()
    message(WARNING "Ipopt not found. Trajectory optimization will be disabled.")
endif()

# ===================== 头文件包含 =====================
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
)

# ===================== 收集所有源码和头文件 =====================
# 收集所有.cpp源码
file(GLOB_RECURSE ALL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/**/*.cpp
)

# 收集所有头文件（include/ + src/下的.h，仅.h文件）
file(GLOB_RECURSE ALL_HEADERS
    ${CMAKE_SOURCE_DIR}/include/**/*.h
    ${CMAKE_SOURCE_DIR}/src/**/*.h
)

# ===================== 仅拷贝头文件到build/include（保留相对路径） =====================
# 1. 创建build/include目录（确保目录存在）
file(MAKE_DIRECTORY ${HEADER_OUTPUT_DIR})

# 2. 自定义目标：仅拷贝.h文件，保留相对路径，不拷贝其他文件
add_custom_target(copy_headers ALL
    COMMENT "Copying header files to ${HEADER_OUTPUT_DIR}..."
)

# 遍历每个头文件，按相对路径拷贝到build/include
foreach(HEADER_FILE ${ALL_HEADERS})
    # 获取头文件相对于项目根目录的路径
    file(RELATIVE_PATH REL_HEADER_PATH ${CMAKE_SOURCE_DIR} ${HEADER_FILE})
    # 拼接目标路径
    set(TARGET_HEADER_PATH ${HEADER_OUTPUT_DIR}/${REL_HEADER_PATH})
    # 创建目标目录（避免路径不存在）
    get_filename_component(TARGET_HEADER_DIR ${TARGET_HEADER_PATH} DIRECTORY)
    file(MAKE_DIRECTORY ${TARGET_HEADER_DIR})
    # 拷贝文件（仅当源文件有修改时拷贝）
    add_custom_command(
        TARGET copy_headers
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${HEADER_FILE}
            ${TARGET_HEADER_PATH}
        COMMENT "Copying ${REL_HEADER_PATH}..."
    )
endforeach()

# ===================== 生成单个静态库 =====================
add_library(j6b_parking_pnc STATIC ${ALL_SOURCES})

# 确保先拷贝头文件，再编译库
add_dependencies(j6b_parking_pnc copy_headers)

# 链接依赖库
target_link_libraries(j6b_parking_pnc
    PUBLIC
        yaml-cpp
    PRIVATE
        $<$<BOOL:${OSQP_FOUND}>:OSQP>
        $<$<BOOL:${Ipopt_FOUND}>:Ipopt>
)

# 设置库属性
set_target_properties(j6b_parking_pnc PROPERTIES
    OUTPUT_NAME j6b_parking_pnc
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# ===================== 测试/示例支持 =====================
enable_testing()
if(EXISTS ${CMAKE_SOURCE_DIR}/test/CMakeLists.txt)
    add_subdirectory(test)
endif()
if(EXISTS ${CMAKE_SOURCE_DIR}/example/CMakeLists.txt)
    add_subdirectory(example)
endif()