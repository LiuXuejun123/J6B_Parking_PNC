# TrajectorySmoothing模块 - 轨迹平滑
add_library(trajectory_smoothing STATIC
    TrajectorySmoother.cpp
)

target_include_directories(trajectory_smoothing PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(trajectory_smoothing PUBLIC
    common
)

if(OSQP_FOUND)
    target_link_libraries(trajectory_smoothing PUBLIC ${OSQP_LIBRARIES})
    target_include_directories(trajectory_smoothing PRIVATE ${OSQP_INCLUDE_DIRS})
    target_compile_definitions(trajectory_smoothing PRIVATE USE_OSQP)
endif()

# 安装规则
install(TARGETS trajectory_smoothing
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    TrajectorySmoother.h
    DESTINATION include/trajectory_smoothing
)
