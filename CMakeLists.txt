cmake_minimum_required(VERSION 3.12)
project(J6B_Parking_PNC VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 添加CMake模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# 查找依赖库
find_package(PkgConfig REQUIRED)

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)
if(NOT yaml-cpp_FOUND)
    message(FATAL_ERROR "yaml-cpp not found. Please install: sudo apt-get install libyaml-cpp-dev")
endif()

# 查找Eigen3 (可选)
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
    add_definitions(-DUSE_EIGEN)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# 查找OSQP (用于轨迹平滑)
find_package(OSQP QUIET)
if(OSQP_FOUND)
    message(STATUS "OSQP found")
    add_definitions(-DUSE_OSQP)
else()
    message(WARNING "OSQP not found. Trajectory smoothing will be disabled.")
endif()

# 查找Ipopt (用于轨迹优化)
find_package(Ipopt QUIET)
if(Ipopt_FOUND)
    message(STATUS "Ipopt found: ${Ipopt_INCLUDE_DIR}")
    add_definitions(-DUSE_IPOPT)
else()
    message(WARNING "Ipopt not found. Trajectory optimization will be disabled.")
endif()

# 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/src
)

# ============================================
# 构建各个模块
# ============================================

# Common模块 (配置读取器和公共数据类型)
add_subdirectory(src/common)

# GridMap模块 (栅格地图构建)
add_subdirectory(src/grid_map)

# ParkingSpaceEvaluation模块 (车位评估)
add_subdirectory(src/parking_space_evaluation)

# ParkingSpaceRecommendation模块 (车位推荐)
add_subdirectory(src/parking_sapce_recommendation)

# HybridAstar模块 (混合A*路径规划)
add_subdirectory(src/hybrid_astar)

# TrajectorySmoothing模块 (轨迹平滑)
add_subdirectory(src/trajectory_smoothing)

# VelocityPlanning模块 (速度规划)
add_subdirectory(src/velocity_planning)

# TrajectoryMerging模块 (轨迹合并)
add_subdirectory(src/trajectory_merging)

# TrajectoryOptimization模块 (轨迹优化)
add_subdirectory(src/trajectory_optimization)

# ControlTrajectoryOutput模块 (控制轨迹输出)
add_subdirectory(src/trajectory_output)

# ============================================
# 创建聚合库 (可选，方便一次性链接所有模块)
# 注意：至少需要一个源文件，否则CMake会报错
# ============================================
add_library(j6b_parking_pnc_all STATIC
    src/j6b_parking_pnc_all_dummy.cpp
)

target_link_libraries(j6b_parking_pnc_all PUBLIC
    common
    grid_map
    parking_space_evaluation
    parking_space_recommendation
    hybrid_astar
    trajectory_smoothing
    velocity_planning
    trajectory_merging
    trajectory_optimization
    trajectory_output
)

# 设置聚合库的输出名称
set_target_properties(j6b_parking_pnc_all PROPERTIES
    OUTPUT_NAME j6b_parking_pnc
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# ============================================
# 安装规则
# ============================================
install(TARGETS 
    common
    grid_map
    parking_space_evaluation
    parking_space_recommendation
    hybrid_astar
    trajectory_smoothing
    velocity_planning
    trajectory_merging
    trajectory_optimization
    trajectory_output
    j6b_parking_pnc_all
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# ============================================
# 测试支持
# ============================================
enable_testing()
if(EXISTS ${CMAKE_SOURCE_DIR}/test/CMakeLists.txt)
    add_subdirectory(test)
endif()

# ============================================
# 示例程序支持
# ============================================
if(EXISTS ${CMAKE_SOURCE_DIR}/example/CMakeLists.txt)
    add_subdirectory(example)
endif()
