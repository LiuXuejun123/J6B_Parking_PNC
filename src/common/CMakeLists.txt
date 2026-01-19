# Common模块 - 配置读取器和公共数据类型
add_library(common STATIC
    config_reader/Config_Reader.cpp
)

target_include_directories(common PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(common PUBLIC
    yaml-cpp
)

# 安装规则
install(TARGETS common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    config_reader/Config_Reader.h
    datatype/APS_Planning_Datatype.h
    math/math.h
    DESTINATION include/common
)
