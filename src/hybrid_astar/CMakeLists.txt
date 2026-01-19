# HybridAstar模块 - 混合A*路径规划
add_library(hybrid_astar STATIC
    Hybrid_Astar.cpp
)

target_include_directories(hybrid_astar PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(hybrid_astar PUBLIC
    common
    grid_map
)

# 安装规则
install(TARGETS hybrid_astar
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    Hybrid_Astar.h
    DESTINATION include/hybrid_astar
)
