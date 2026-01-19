# TrajectoryMerging模块 - 轨迹合并
add_library(trajectory_merging STATIC
    TrajectoryMerger.cpp
)

target_include_directories(trajectory_merging PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(trajectory_merging PUBLIC
    common
    velocity_planning
)

# 安装规则
install(TARGETS trajectory_merging
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    TrajectoryMerger.h
    DESTINATION include/trajectory_merging
)
