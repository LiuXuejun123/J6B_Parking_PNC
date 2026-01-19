# 自动泊车规划系统使用示例

## 概述

本示例展示了如何编译成库后，在其他代码中调用自动泊车规划系统的各个模块。

## 编译方法

### 1. 编译整个项目（包括库和示例）

```bash
mkdir build
cd build
cmake ..
make
```

编译完成后，库文件将位于 `build/lib/` 目录，示例程序位于 `build/bin/` 目录。

### 2. 只编译库（不编译示例）

```bash
mkdir build
cd build
cmake -DBUILD_EXAMPLES=OFF ..
make
```

## 运行示例

```bash
cd build/bin
./parking_planning_example [配置文件路径]
```

如果不指定配置文件路径，默认使用 `../config/aps_planning_config.yaml`

## 在其他项目中使用本库

### 方法1: 使用安装的库（推荐）

首先安装库到系统目录：

```bash
cd build
make install
```

然后在你的项目的 `CMakeLists.txt` 中：

```cmake
# 查找库
find_library(J6B_PARKING_PNC_LIB 
    NAMES j6b_parking_pnc
    PATHS /usr/local/lib
          ${CMAKE_INSTALL_PREFIX}/lib
)

# 查找头文件
find_path(J6B_PARKING_PNC_INCLUDE_DIR
    NAMES common/config_reader/Config_Reader.h
    PATHS /usr/local/include
          ${CMAKE_INSTALL_PREFIX}/include
)

# 链接库
target_link_libraries(your_target PRIVATE
    ${J6B_PARKING_PNC_LIB}
    yaml-cpp
)

# 包含头文件
target_include_directories(your_target PRIVATE
    ${J6B_PARKING_PNC_INCLUDE_DIR}
)
```

### 方法2: 直接链接编译后的库

在你的项目的 `CMakeLists.txt` 中：

```cmake
# 假设库编译在 ../J6B_Parking_PNC/build/lib
set(J6B_PARKING_PNC_LIB_DIR "${CMAKE_SOURCE_DIR}/../J6B_Parking_PNC/build/lib")
set(J6B_PARKING_PNC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../J6B_Parking_PNC/src")

# 链接库
target_link_libraries(your_target PRIVATE
    ${J6B_PARKING_PNC_LIB_DIR}/libj6b_parking_pnc.a
    yaml-cpp
)

# 包含头文件
target_include_directories(your_target PRIVATE
    ${J6B_PARKING_PNC_INCLUDE_DIR}
)
```

### 方法3: 使用CMake的add_subdirectory

在你的项目的 `CMakeLists.txt` 中：

```cmake
# 添加子目录
add_subdirectory(../J6B_Parking_PNC J6B_Parking_PNC_build)

# 链接库
target_link_libraries(your_target PRIVATE
    j6b_parking_pnc_all
    yaml-cpp
)

# 包含头文件会自动处理
```

## 使用流程

1. **初始化配置读取器**：创建 `Config_Reader` 实例并加载配置文件
2. **车位评估**：使用 `ParkingSpaceEvaluator` 评估车位有效性
3. **车位推荐**：使用 `ParkingSpaceRecommender` 推荐最适合的车位
4. **栅格地图构建**：使用 `GridMapBuilder` 构建栅格地图
5. **路径规划**：使用 `Hybrid_Astar` 规划路径
6. **轨迹平滑**：使用 `TrajectorySmoother` 平滑轨迹
7. **速度规划**：使用 `VelocityPlanner` 规划速度
8. **轨迹合并**：使用 `TrajectoryMerger` 合并位置和速度
9. **轨迹优化**：使用 `TrajectoryOptimizer` 优化轨迹
10. **控制轨迹输出**：使用 `ControlTrajectoryOutput` 输出最终轨迹

## 模块依赖关系

```
common (配置读取器)
├── grid_map
├── parking_space_evaluation
├── parking_space_recommendation (依赖 parking_space_evaluation)
├── hybrid_astar (依赖 grid_map)
├── trajectory_smoothing
├── velocity_planning
├── trajectory_merging (依赖 velocity_planning)
├── trajectory_optimization
└── trajectory_output
```

## 注意事项

1. 所有模块都依赖 `common` 模块（配置读取器）
2. 确保配置文件路径正确
3. 如果使用 OSQP 或 Ipopt，需要确保这些库已正确安装
4. 建议使用聚合库 `j6b_parking_pnc_all` 来简化链接
