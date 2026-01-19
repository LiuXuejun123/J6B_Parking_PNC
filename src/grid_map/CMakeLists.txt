# GridMap模块 - 栅格地图构建
add_library(grid_map STATIC
    GridMapBuilder.cpp
)

target_include_directories(grid_map PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(grid_map PUBLIC
    common
)

# 安装规则
install(TARGETS grid_map
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    GridMapBuilder.h
    DESTINATION include/grid_map
)
