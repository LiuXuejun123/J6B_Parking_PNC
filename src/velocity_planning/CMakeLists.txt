# VelocityPlanning模块 - 速度规划
add_library(velocity_planning STATIC
    VelocityPlanner.cpp
)

target_include_directories(velocity_planning PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(velocity_planning PUBLIC
    common
)

# 安装规则
install(TARGETS velocity_planning
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    VelocityPlanner.h
    DESTINATION include/velocity_planning
)
