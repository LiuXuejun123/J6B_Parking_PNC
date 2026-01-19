cmake_minimum_required(VERSION 3.12)
project(J6M_Parking_PDC VERSION 1.0.0 LANGUAGES CXX)

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
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# 源文件列表
set(COMMON_SOURCES
    src/config/config_reader.cpp
    src/mapping/data_mapper.cpp
)

set(PARKING_SPACE_EVAL_SOURCES
    src/parking_space_evaluation/parking_space_evaluator.cpp
)

set(PARKING_SPACE_REC_SOURCES
    src/parking_space_recommendation/parking_space_recommender.cpp
)

set(GRID_MAP_SOURCES
    src/grid_map/grid_map_builder.cpp
)

set(HYBRID_ASTAR_SOURCES
    src/hybrid_astar/hybrid_astar_planner.cpp
)

set(TRAJECTORY_SMOOTHING_SOURCES
    src/trajectory_smoothing/trajectory_smoother.cpp
)

set(VELOCITY_PLANNING_SOURCES
    src/velocity_planning/velocity_planner.cpp
)

set(TRAJECTORY_MERGING_SOURCES
    src/trajectory_merging/trajectory_merger.cpp
)

set(TRAJECTORY_OPTIMIZATION_SOURCES
    src/trajectory_optimization/trajectory_optimizer.cpp
)

set(CONTROL_TRAJECTORY_OUTPUT_SOURCES
    src/control_trajectory_output/control_trajectory_output.cpp
        src/common/config_reader/Config_Reader.cpp
        src/common/config_reader/Config_Reader.h
        src/common/datatype/APS_Planning_Datatype.h
        src/common/math/math.h
        src/common/math/math.h
        src/grid_map/GridMapBuilder.cpp
        src/grid_map/GridMapBuilder.h
        src/mapping/Mapping.h
        src/hybrid_astar/Hybrid_Astar.cpp
        src/hybrid_astar/Hybrid_Astar.h
        src/parking_sapce_recommendation/ParkingSpaceRecommender.cpp
        src/parking_sapce_recommendation/ParkingSpaceRecommender.h
        src/parking_space_evaluation/ParkingSpaceEvaluator.cpp
        src/parking_space_evaluation/ParkingSpaceEvaluator.h
        src/trajectory_merging/TrajectoryMerger.cpp
        src/trajectory_merging/TrajectoryMerger.h
        src/trajectory_optimization/TrajectoryOptimizer.cpp
        src/trajectory_optimization/TrajectoryOptimizer.h
        src/trajectory_output/ControlTrajectoryOutput.cpp
        src/trajectory_output/ControlTrajectoryOutput.h
        src/trajectory_smoothing/TrajectorySmoother.cpp
        src/trajectory_smoothing/TrajectorySmoother.h
        src/velocity_planning/VelocityPlanner.cpp
        src/velocity_planning/VelocityPlanner.h
        test/Test.h
)

# 创建静态库
add_library(j6m_parking_pdc_static STATIC
    ${COMMON_SOURCES}
    ${PARKING_SPACE_EVAL_SOURCES}
    ${PARKING_SPACE_REC_SOURCES}
    ${GRID_MAP_SOURCES}
    ${HYBRID_ASTAR_SOURCES}
    ${TRAJECTORY_SMOOTHING_SOURCES}
    ${VELOCITY_PLANNING_SOURCES}
    ${TRAJECTORY_MERGING_SOURCES}
    ${TRAJECTORY_OPTIMIZATION_SOURCES}
    ${CONTROL_TRAJECTORY_OUTPUT_SOURCES}
)

# 链接库
target_link_libraries(j6m_parking_pdc_static
    yaml-cpp
)

if(Eigen3_FOUND)
    target_link_libraries(j6m_parking_pdc_static Eigen3::Eigen)
endif()

if(OSQP_FOUND)
    target_link_libraries(j6m_parking_pdc_static ${OSQP_LIBRARIES})
    target_include_directories(j6m_parking_pdc_static PRIVATE ${OSQP_INCLUDE_DIRS})
endif()

if(Ipopt_FOUND)
    target_link_libraries(j6m_parking_pdc_static ${Ipopt_LIBRARIES})
    target_include_directories(j6m_parking_pdc_static PRIVATE ${Ipopt_INCLUDE_DIRS})
endif()

# 设置库的输出名称
set_target_properties(j6m_parking_pdc_static PROPERTIES
    OUTPUT_NAME j6m_parking_pdc
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 安装规则
install(TARGETS j6m_parking_pdc_static
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# 测试支持
enable_testing()
add_subdirectory(test)

# 示例程序支持
add_subdirectory(examples)
