cmake_minimum_required(VERSION 3.28)
project(J6B_Parking_Planning)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)

add_library(J6B_Parking_Planning STATIC

        # 只放 .cpp 來源檔（.h 不需要寫在這裡）
        J6B_Parking_PNC.cpp
        src/ControlTrajectoryOutput.cpp
        src/GridMapBuilder.cpp
        src/Hybrid_Astar.cpp
        src/ParkingSpaceEvaluator.cpp
        src/ParkingSpaceRecommender.cpp
        src/TrajectoryMerger.cpp
        src/TrajectoryOptimizer.cpp
        src/TrajectorySmoother.cpp
        src/VelocityPlanner.cpp
        src/Planning_Math.cpp
        src/Config_Reader.cpp
)

# 最重要的這一行：讓所有使用這個 library 的目標都能找到頭文件
target_include_directories(J6B_Parking_Planning
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # 如果有只有這個 library 內部才需要的私有頭文件，可以再加 PRIVATE 路徑
        # PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(J6B_Parking_Planning
        PUBLIC
        yaml-cpp::yaml-cpp
)


# 快速把头文件复制到 build/include（可选）
add_custom_command(
        TARGET J6B_Parking_Planning POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include
        # 第一步：复制include目录的所有头文件（原有逻辑）
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include
        # 第二步：单独复制根目录下的J6B_Parking_PNC.h（新增）
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/J6B_Parking_PNC.h ${CMAKE_BINARY_DIR}/include/
        COMMENT "Copying header files to build/include (including J6B_Parking_PNC.h)"
)


# =========================
# GTest 单元测试配置
# =========================

# 使用系统中已安装的 GTest
find_package(GTest REQUIRED)
if(GTest_FOUND)
    message(STATUS "GTest found, building tests")

    enable_testing()

    add_executable(planning_tests
            tests/test_PlanningMath.cpp
            tests/test_ParkingSpaceEvaluator.cpp
            tests/test_ParkingSpaceRecommender.cpp
            tests/test_HybridAstar.cpp
            tests/test_ControlTrajectoryOutput.cpp
            tests/test_J6B_Parking_PNC.cpp
    )

    target_link_libraries(planning_tests
            PRIVATE
            J6B_Parking_Planning
            GTest::gtest_main
    )

    target_include_directories(planning_tests
            PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    include(GoogleTest)
    gtest_discover_tests(planning_tests)
else()
    message(STATUS "GTest not found, skipping tests")
endif()
