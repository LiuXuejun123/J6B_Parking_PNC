# 构建和使用指南

## 项目结构

本项目采用模块化设计，每个模块都有独立的 `CMakeLists.txt`，最外层的 `CMakeLists.txt` 负责汇总所有模块。

```
J6B_Parking_PNC/
├── CMakeLists.txt                    # 主CMake文件（汇总所有模块）
├── src/
│   ├── common/
│   │   ├── CMakeLists.txt           # Common模块CMake
│   │   ├── config_reader/           # 配置读取器（所有模块共用）
│   │   ├── datatype/                # 公共数据类型
│   │   └── math/                    # 数学工具
│   ├── grid_map/
│   │   ├── CMakeLists.txt           # GridMap模块CMake
│   │   ├── GridMapBuilder.h
│   │   └── GridMapBuilder.cpp
│   ├── parking_space_evaluation/
│   │   ├── CMakeLists.txt
│   │   ├── ParkingSpaceEvaluator.h
│   │   └── ParkingSpaceEvaluator.cpp
│   └── ... (其他模块类似)
├── example/
│   ├── CMakeLists.txt               # 示例程序CMake
│   ├── parking_planning_example.cpp # 使用示例
│   └── README.md                    # 示例说明
└── config/
    └── aps_planning_config.yaml     # 配置文件
```

## 编译步骤

### 1. 基本编译

```bash
mkdir build
cd build
cmake ..
make
```

### 2. 编译输出

编译完成后，会生成以下文件：

- **库文件**（位于 `build/lib/`）：
  - `libcommon.a` - 公共模块（配置读取器）
  - `libgrid_map.a` - 栅格地图模块
  - `libparking_space_evaluation.a` - 车位评估模块
  - `libparking_space_recommendation.a` - 车位推荐模块
  - `libhybrid_astar.a` - Hybrid A*模块
  - `libtrajectory_smoothing.a` - 轨迹平滑模块
  - `libvelocity_planning.a` - 速度规划模块
  - `libtrajectory_merging.a` - 轨迹合并模块
  - `libtrajectory_optimization.a` - 轨迹优化模块
  - `libtrajectory_output.a` - 控制轨迹输出模块
  - `libj6b_parking_pnc.a` - **聚合库（推荐使用）**，包含所有模块

- **可执行文件**（位于 `build/bin/`）：
  - `parking_planning_example` - 示例程序

### 3. 安装库（可选）

```bash
cd build
make install
```

默认安装路径：
- 库文件：`/usr/local/lib/`
- 头文件：`/usr/local/include/`

## 在其他项目中使用

### 方法1：使用聚合库（推荐）

最简单的方式是链接聚合库 `j6b_parking_pnc`，它会自动包含所有依赖。

```cmake
# 在你的项目的CMakeLists.txt中

# 假设库在 ../J6B_Parking_PNC/build/lib
set(J6B_PARKING_PNC_LIB_DIR "${CMAKE_SOURCE_DIR}/../J6B_Parking_PNC/build/lib")
set(J6B_PARKING_PNC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../J6B_Parking_PNC/src")

# 创建你的目标
add_executable(your_program main.cpp)

# 链接聚合库
target_link_libraries(your_program PRIVATE
    ${J6B_PARKING_PNC_LIB_DIR}/libj6b_parking_pnc.a
    yaml-cpp
)

# 包含头文件
target_include_directories(your_program PRIVATE
    ${J6B_PARKING_PNC_INCLUDE_DIR}
)
```

### 方法2：使用CMake的add_subdirectory

```cmake
# 在你的项目的CMakeLists.txt中

# 添加子目录
add_subdirectory(../J6B_Parking_PNC J6B_Parking_PNC_build)

# 创建你的目标
add_executable(your_program main.cpp)

# 链接聚合库
target_link_libraries(your_program PRIVATE
    j6b_parking_pnc_all
    yaml-cpp
)
```

### 方法3：只链接需要的模块

如果只需要部分功能，可以只链接相关模块：

```cmake
target_link_libraries(your_program PRIVATE
    common                    # 必需：配置读取器
    grid_map
    hybrid_astar
    yaml-cpp
)
```

## 使用示例

### 基本使用流程

```cpp
#include "common/config_reader/Config_Reader.h"
#include "grid_map/GridMapBuilder.h"
#include "hybrid_astar/Hybrid_Astar.h"

using namespace APS_Planning;
using namespace APS_Planning::common;

int main() {
    // 1. 初始化配置读取器
    Config_Reader config_reader("../config/aps_planning_config.yaml");
    config_reader.read_grid_map_config();
    
    // 2. 获取配置
    auto grid_map_cfg = config_reader.get_grid_map_config();
    
    // 3. 使用各个模块
    GridMapBuilder builder;
    Hybrid_Astar planner;
    
    // ... 调用模块功能
    
    return 0;
}
```

详细示例请参考 `example/parking_planning_example.cpp`

## 模块依赖关系

```
common (配置读取器，所有模块的基础)
│
├── grid_map
│   └── hybrid_astar (依赖 grid_map)
│
├── parking_space_evaluation
│   └── parking_space_recommendation (依赖 parking_space_evaluation)
│
├── trajectory_smoothing
│
├── velocity_planning
│   └── trajectory_merging (依赖 velocity_planning)
│
├── trajectory_optimization
│
└── trajectory_output
```

## 配置说明

所有模块共享一个配置文件：`config/aps_planning_config.yaml`

配置读取器 `Config_Reader` 负责读取和解析配置，各模块通过 `Config_Reader` 获取自己的配置参数。

## 注意事项

1. **配置文件路径**：确保配置文件路径正确，或使用绝对路径
2. **依赖库**：
   - `yaml-cpp`：必需
   - `Eigen3`：可选（如果使用）
   - `OSQP`：可选（轨迹平滑需要）
   - `Ipopt`：可选（轨迹优化需要）
3. **C++标准**：项目使用 C++17 标准
4. **编译选项**：默认使用 `-O3` 优化，Debug模式使用 `-g -O0`

## 故障排除

### 问题1：找不到配置文件

**解决方案**：使用绝对路径或确保相对路径正确

```cpp
Config_Reader config_reader("/absolute/path/to/config/aps_planning_config.yaml");
```

### 问题2：链接错误

**解决方案**：确保链接了所有依赖库，包括 `yaml-cpp`

### 问题3：头文件找不到

**解决方案**：确保 `target_include_directories` 包含了 `src` 目录

```cmake
target_include_directories(your_target PRIVATE
    ${CMAKE_SOURCE_DIR}/../J6B_Parking_PNC/src
)
```
