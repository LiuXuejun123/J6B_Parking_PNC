# 对外交付指南

## 交付内容

对外交付时，只需要提供以下内容：

1. **头文件**：`include/j6b_parking_pnc/J6BParkingPlanner.h`
2. **静态库**：`libj6b_parking_pnc.a`
3. **配置文件示例**：`config/aps_planning_config.yaml`（可选，用于参考）

## 快速开始

### 1. 基本使用

```cpp
#include "j6b_parking_pnc/J6BParkingPlanner.h"
#include <iostream>

int main() {
    // 创建规划器实例
    APS_Planning::J6BParkingPlanner planner;
    
    // 初始化（加载配置文件）
    if (!planner.Initialize("config/aps_planning_config.yaml")) {
        std::cerr << "Failed to initialize planner" << std::endl;
        return -1;
    }
    
    // 定义起始和目标位姿 [x, y, theta]
    double start_pose[3] = {0.0, 0.0, 0.0};
    double goal_pose[3] = {10.0, 5.0, 1.57};
    
    // 执行完整规划流程
    void* final_trajectory = nullptr; // 使用实际的数据结构
    if (planner.PlanParkingTrajectory(
            start_pose, 
            goal_pose, 
            nullptr,  // obstacles
            nullptr,  // freespace
            final_trajectory)) {
        std::cout << "Planning successful!" << std::endl;
    } else {
        std::cerr << "Planning failed!" << std::endl;
    }
    
    return 0;
}
```

### 2. 分步骤使用

```cpp
// 1. 构建栅格地图
planner.BuildGridMap(obstacles, freespace);

// 2. 路径规划
void* path = nullptr;
planner.PlanPath(start_pose, goal_pose, path);

// 3. 轨迹平滑
void* smoothed = nullptr;
planner.SmoothTrajectory(path, smoothed);

// 4. 速度规划
void* velocity = nullptr;
planner.PlanVelocity(smoothed, velocity);

// 5. 轨迹合并
void* merged = nullptr;
planner.MergeTrajectory(smoothed, velocity, merged);

// 6. 轨迹优化
void* optimized = nullptr;
planner.OptimizeTrajectory(merged, optimized);

// 7. 获取控制轨迹
double current_pose[3] = {1.0, 1.0, 0.5};
void* control = nullptr;
planner.GetControlTrajectory(optimized, current_pose, control);
```

### 3. 使用各个模块的独立功能

```cpp
// 车位评估
bool is_valid = planner.EvaluateParkingSpace(parking_space_data);

// 车位推荐
std::vector<int> recommended = planner.RecommendParkingSpaces(parking_spaces, 3);

// 直接访问底层模块（高级用法）
auto* hybrid_astar = planner.GetHybridAstar();
auto* smoother = planner.GetTrajectorySmoother();
// ... 使用底层模块的接口
```

## CMake集成

在你的项目的 `CMakeLists.txt` 中：

```cmake
# 设置库路径
set(J6B_PARKING_PNC_LIB_DIR "${CMAKE_SOURCE_DIR}/path/to/j6b_parking_pnc/lib")
set(J6B_PARKING_PNC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/path/to/j6b_parking_pnc/include")

# 创建你的目标
add_executable(your_program main.cpp)

# 链接库
target_link_libraries(your_program PRIVATE
    ${J6B_PARKING_PNC_LIB_DIR}/libj6b_parking_pnc.a
    yaml-cpp  # 必需依赖
)

# 包含头文件
target_include_directories(your_program PRIVATE
    ${J6B_PARKING_PNC_INCLUDE_DIR}
)
```

## 依赖要求

- **C++标准**：C++17 或更高
- **必需库**：yaml-cpp
- **可选库**：
  - Eigen3（如果使用矩阵运算）
  - OSQP（轨迹平滑需要）
  - Ipopt（轨迹优化需要）

## 接口说明

### 主要接口类

`J6BParkingPlanner` 是唯一的对外接口类，封装了所有功能模块。

### 核心方法

1. **Initialize()** - 初始化规划器，必须在使用前调用
2. **PlanParkingTrajectory()** - 一键执行完整规划流程（推荐使用）
3. **各模块独立接口** - 可以分步骤调用各个模块

### 数据格式

当前接口使用 `void*` 作为数据指针，具体的数据结构需要根据实际需求定义。建议：

1. 定义统一的数据结构（如 `Trajectory`, `ParkingSpace` 等）
2. 在接口实现中转换为内部数据结构
3. 或者直接使用内部数据结构

## 注意事项

1. **初始化顺序**：必须先调用 `Initialize()` 才能使用其他接口
2. **线程安全**：当前实现不是线程安全的，多线程使用需要加锁
3. **内存管理**：接口类使用智能指针管理资源，无需手动释放
4. **配置文件**：确保配置文件路径正确，或使用绝对路径

## 示例项目结构

```
your_project/
├── CMakeLists.txt
├── main.cpp
├── j6b_parking_pnc/
│   ├── include/
│   │   └── j6b_parking_pnc/
│   │       └── J6BParkingPlanner.h
│   └── lib/
│       └── libj6b_parking_pnc.a
└── config/
    └── aps_planning_config.yaml
```

## 技术支持

如有问题，请参考：
- `BUILD_GUIDE.md` - 构建指南
- `example/parking_planning_example.cpp` - 完整示例代码
