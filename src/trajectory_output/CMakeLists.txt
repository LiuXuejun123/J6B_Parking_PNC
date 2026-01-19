# ControlTrajectoryOutput模块 - 控制轨迹输出
add_library(trajectory_output STATIC
    ControlTrajectoryOutput.cpp
)

target_include_directories(trajectory_output PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(trajectory_output PUBLIC
    common
)

# 安装规则
install(TARGETS trajectory_output
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    ControlTrajectoryOutput.h
    DESTINATION include/trajectory_output
)
