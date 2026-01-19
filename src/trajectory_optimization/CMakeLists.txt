# TrajectoryOptimization模块 - 轨迹优化
add_library(trajectory_optimization STATIC
    TrajectoryOptimizer.cpp
)

target_include_directories(trajectory_optimization PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(trajectory_optimization PUBLIC
    common
)

if(Ipopt_FOUND)
    target_link_libraries(trajectory_optimization PUBLIC ${Ipopt_LIBRARIES})
    target_include_directories(trajectory_optimization PRIVATE ${Ipopt_INCLUDE_DIRS})
    target_compile_definitions(trajectory_optimization PRIVATE USE_IPOPT)
endif()

# 安装规则
install(TARGETS trajectory_optimization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    TrajectoryOptimizer.h
    DESTINATION include/trajectory_optimization
)
