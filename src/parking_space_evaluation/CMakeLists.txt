# ParkingSpaceEvaluation模块 - 车位评估
add_library(parking_space_evaluation STATIC
    ParkingSpaceEvaluator.cpp
)

target_include_directories(parking_space_evaluation PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(parking_space_evaluation PUBLIC
    common
)

# 安装规则
install(TARGETS parking_space_evaluation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    ParkingSpaceEvaluator.h
    DESTINATION include/parking_space_evaluation
)
