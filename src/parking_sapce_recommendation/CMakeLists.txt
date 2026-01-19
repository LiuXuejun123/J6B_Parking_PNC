# ParkingSpaceRecommendation模块 - 车位推荐
add_library(parking_space_recommendation STATIC
    ParkingSpaceRecommender.cpp
)

target_include_directories(parking_space_recommendation PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(parking_space_recommendation PUBLIC
    common
    parking_space_evaluation
)

# 安装规则
install(TARGETS parking_space_recommendation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES
    ParkingSpaceRecommender.h
    DESTINATION include/parking_space_recommendation
)
