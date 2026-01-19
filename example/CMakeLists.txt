# 示例程序 - 展示如何使用编译后的库

# 创建示例可执行文件
add_executable(parking_planning_example
    parking_planning_example.cpp
)

# 链接所有需要的库
target_link_libraries(parking_planning_example PRIVATE
    j6b_parking_pnc_all  # 使用聚合库，一次性链接所有模块
    # 或者单独链接需要的模块：
    # common
    # grid_map
    # parking_space_evaluation
    # parking_space_recommendation
    # hybrid_astar
    # trajectory_smoothing
    # velocity_planning
    # trajectory_merging
    # trajectory_optimization
    # trajectory_output
)

# 设置可执行文件输出目录
set_target_properties(parking_planning_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 安装示例程序（可选）
install(TARGETS parking_planning_example
    RUNTIME DESTINATION bin
)
