# Core模块 - 统一接口类
add_library(core STATIC
    J6BParkingPlanner.cpp
)

target_include_directories(core PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(core PUBLIC
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

# 安装规则
install(TARGETS core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    J6BParkingPlanner.h
    DESTINATION include/core
)
