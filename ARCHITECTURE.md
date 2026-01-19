# J6M 自动泊车规控系统 - 工程架构设计文档

## 一、模块依赖关系分析

### 1.1 模块依赖图
```
外部输入（感知数据）
    ↓
[Mapping Layer] ← 数据格式转换层（静态库）
    ↓
[Config Module] ← 配置管理（所有模块共享）
    ↓
┌─────────────────────────────────────────────────────┐
│              功能模块层                               │
├─────────────────────────────────────────────────────┤
│                                                       │
│  [Parking Space Evaluation] 车位评估                 │
│         ↓                                            │
│  [Parking Space Recommendation] 车位推荐             │
│         ↓                                            │
│  [Grid Map Builder] 栅格地图构建                     │
│         ↓                                            │
│  [Hybrid A* Planner] 路径规划                        │
│         ↓                                            │
│  [Trajectory Smoother] 轨迹平滑                      │
│         ↓                                            │
│  [Velocity Planner] 速度规划                         │
│         ↓                                            │
│  [Trajectory Merger] 轨迹合并                        │
│         ↓                                            │
│  [Trajectory Optimizer] 轨迹优化                     │
│         ↓                                            │
│  [Control Trajectory Output] 控制轨迹输出            │
│                                                       │
└─────────────────────────────────────────────────────┘
```

### 1.2 依赖层次说明

**第0层：基础支撑层**
- `common/`: 公共数据类型（Point2D, Point3D, Trajectory, ParkingSpace等）
- `config/`: 配置管理模块（ConfigReader类）
- `mapping/`: 数据映射层（内部↔外部数据转换）

**第1层：独立功能模块**
- `parking_space_evaluation/`: 车位评估（依赖：common, config）
- `parking_space_recommendation/`: 车位推荐（依赖：common, config, parking_space_evaluation）

**第2层：地图与规划层**
- `grid_map/`: 栅格地图构建（依赖：common, config, mapping）
- `hybrid_astar/`: Hybrid A*路径规划（依赖：common, config, grid_map）

**第3层：轨迹处理层**
- `trajectory_smoothing/`: 轨迹平滑（依赖：common, config, hybrid_astar, OSQP）
- `velocity_planning/`: 速度规划（依赖：common, config）
- `trajectory_merging/`: 轨迹合并（依赖：common, config, trajectory_smoothing, velocity_planning）

**第4层：优化与输出层**
- `trajectory_optimization/`: 轨迹优化（依赖：common, config, trajectory_merging, Ipopt）
- `control_trajectory_output/`: 控制轨迹输出（依赖：common, config, trajectory_optimization, mapping）

## 二、推荐文件夹结构

```
J6M_Parking_PDC/
├── CMakeLists.txt                      # 主CMake构建文件
├── README.md                           # 项目说明
├── ARCHITECTURE.md                     # 架构设计文档（本文档）
│
├── cmake/                              # CMake辅助模块
│   ├── FindIpopt.cmake                # Ipopt库查找模块
│   ├── FindOSQP.cmake                 # OSQP库查找模块
│   └── CompilerOptions.cmake          # 编译器选项设置（可选）
│
├── include/                            # 公共头文件目录（对外接口）
│   ├── common/                         # 公共数据类型
│   │   ├── types.h                    # 基础类型定义（GridMap, GridState等）
│   │   ├── point.h                    # 点类型（Point2D, Point3D）
│   │   ├── trajectory.h               # 轨迹类型（Trajectory, TrajectoryPoint）
│   │   ├── parking_space.h            # 车位类型（ParkingSpace）
│   │   └── obstacle.h                 # 障碍物类型（可选）
│   │
│   ├── config/                         # 配置管理接口
│   │   ├── config_reader.h            # ConfigReader类接口
│   │   └── config_structs.h           # 配置结构体定义（可选）
│   │
│   ├── mapping/                        # 数据映射层接口
│   │   └── data_mapper.h              # DataMapper类接口
│   │
│   ├── parking_space_evaluation/      # 车位评估模块接口
│   │   └── parking_space_evaluator.h
│   │
│   ├── parking_space_recommendation/  # 车位推荐模块接口
│   │   └── parking_space_recommender.h
│   │
│   ├── grid_map/                      # 栅格地图构建模块接口
│   │   └── grid_map_builder.h
│   │
│   ├── hybrid_astar/                  # Hybrid A*算法模块接口
│   │   └── hybrid_astar_planner.h
│   │
│   ├── trajectory_smoothing/          # 轨迹平滑模块接口
│   │   └── trajectory_smoother.h
│   │
│   ├── velocity_planning/             # 速度规划模块接口
│   │   └── velocity_planner.h
│   │
│   ├── trajectory_merging/            # 轨迹合并模块接口
│   │   └── trajectory_merger.h
│   │
│   ├── trajectory_optimization/       # 轨迹优化模块接口
│   │   └── trajectory_optimizer.h
│   │
│   └── control_trajectory_output/     # 控制轨迹输出模块接口
│       └── control_trajectory_output.h
│
├── src/                                # 源代码实现目录
│   ├── common/                         # 公共类型实现（如果头文件中没有内联实现）
│   │   └── types.cpp                  # GridMap等方法实现
│   │
│   ├── config/                         # 配置管理实现
│   │   ├── config_reader.cpp          # ConfigReader实现
│   │   └── config_reader_impl.h       # 内部实现细节（可选）
│   │
│   ├── mapping/                        # 数据映射层实现
│   │   ├── data_mapper.cpp            # DataMapper实现
│   │   └── internal_types.h           # 内部数据结构定义（如果需要）
│   │
│   ├── parking_space_evaluation/      # 车位评估模块实现
│   │   ├── parking_space_evaluator.cpp
│   │   └── parking_space_evaluator_impl.h  # 内部辅助函数（可选）
│   │
│   ├── parking_space_recommendation/  # 车位推荐模块实现
│   │   ├── parking_space_recommender.cpp
│   │   └── scoring_strategy.h         # 评分策略（可选，如果需要多种策略）
│   │
│   ├── grid_map/                      # 栅格地图构建模块实现
│   │   ├── grid_map_builder.cpp
│   │   └── line_to_grid_converter.h   # 线段转栅格转换器（可选）
│   │
│   ├── hybrid_astar/                  # Hybrid A*算法模块实现
│   │   ├── hybrid_astar_planner.cpp
│   │   ├── node.h                     # A*节点定义
│   │   └── heuristics.h               # 启发式函数（可选）
│   │
│   ├── trajectory_smoothing/          # 轨迹平滑模块实现
│   │   ├── trajectory_smoother.cpp
│   │   └── osqp_wrapper.h             # OSQP封装（可选）
│   │
│   ├── velocity_planning/             # 速度规划模块实现
│   │   ├── velocity_planner.cpp
│   │   └── velocity_profile.h         # 速度曲线类型（梯形、正弦等）
│   │
│   ├── trajectory_merging/            # 轨迹合并模块实现
│   │   └── trajectory_merger.cpp
│   │
│   ├── trajectory_optimization/       # 轨迹优化模块实现
│   │   ├── trajectory_optimizer.cpp
│   │   └── ipopt_wrapper.h            # Ipopt封装（可选）
│   │
│   └── control_trajectory_output/     # 控制轨迹输出模块实现
│       └── control_trajectory_output.cpp
│
├── config/                             # YAML配置文件目录
│   ├── common.yaml                     # 公共配置（如地图分辨率等）
│   ├── parking_space_evaluation.yaml   # 车位评估参数
│   ├── parking_space_recommendation.yaml # 车位推荐参数
│   ├── grid_map.yaml                   # 栅格地图构建参数
│   ├── hybrid_astar.yaml               # Hybrid A*算法参数
│   ├── trajectory_smoothing.yaml       # 轨迹平滑参数
│   ├── velocity_planning.yaml          # 速度规划参数
│   ├── trajectory_optimization.yaml    # 轨迹优化参数
│   └── control_trajectory_output.yaml  # 控制轨迹输出参数
│
├── test/                               # 单元测试目录
│   ├── CMakeLists.txt                  # 测试CMake配置
│   ├── test_common/                    # 测试公共工具
│   │   └── test_utils.h
│   ├── test_config/                    # 配置模块测试
│   │   └── test_config_reader.cpp
│   ├── test_mapping/                   # 映射层测试
│   │   └── test_data_mapper.cpp
│   ├── test_parking_space_evaluation/  # 车位评估测试
│   │   └── test_parking_space_evaluator.cpp
│   └── ...                             # 其他模块测试
│
├── examples/                           # 示例代码目录
│   ├── CMakeLists.txt                  # 示例CMake配置
│   ├── basic_usage.cpp                 # 基础使用示例
│   ├── full_pipeline.cpp               # 完整流程示例
│   └── module_usage/                   # 各模块独立使用示例
│       ├── example_parking_evaluation.cpp
│       ├── example_hybrid_astar.cpp
│       └── ...
│
└── docs/                               # 文档目录（可选）
    ├── API.md                          # API文档
    ├── MODULES.md                      # 模块说明文档
    └── CONFIG.md                       # 配置参数说明文档
```

## 三、各模块接口设计要点

### 3.1 Config模块（config/config_reader.h）
```cpp
class ConfigReader {
    // 单例模式或工厂模式
    // 使用智能指针管理
    // 提供YAML配置读取接口
    // 各模块通过模块名获取配置段
};
```

### 3.2 Mapping模块（mapping/data_mapper.h）
```cpp
class DataMapper {
    // 外部感知数据 → 内部数据结构
    // 内部轨迹数据 → 外部控制数据
    // 提供静态方法或单例
};
```

### 3.3 各功能模块接口
- 每个模块提供独立的类
- 构造函数接收 `ConfigReader` 的智能指针
- 提供主要的 `Plan()`, `Evaluate()`, `Recommend()` 等方法
- 输入输出使用 `common/` 中定义的数据类型

## 四、配置文件组织

### 4.1 配置文件命名规范
- 每个模块对应一个YAML文件
- 文件名为模块名（小写+下划线）
- 使用层级结构组织参数

### 4.2 YAML配置示例结构
```yaml
# config/parking_space_evaluation.yaml
parking_space_evaluation:
  min_width: 2.3          # 最小宽度（米）
  max_width: 3.5          # 最大宽度（米）
  min_length: 4.5         # 最小长度（米）
  max_angle_diff: 0.174   # 最大角度差（弧度，约10度）
  
# config/hybrid_astar.yaml
hybrid_astar:
  resolution:
    x: 0.1               # X方向分辨率（米）
    y: 0.1               # Y方向分辨率（米）
    theta: 0.087         # 角度分辨率（弧度，约5度）
  vehicle_params:
    length: 4.8          # 车长（米）
    width: 1.8           # 车宽（米）
```

## 五、CMake构建组织

### 5.1 静态库结构
- 将所有模块编译为一个统一的静态库：`libj6m_parking_pdc.a`
- 或者按模块拆分为多个静态库（推荐统一库，便于使用）

### 5.2 依赖管理
- OSQP: 仅轨迹平滑模块需要
- Ipopt: 仅轨迹优化模块需要
- yaml-cpp: Config和Mapping模块需要
- Eigen3: 推荐使用，用于矩阵运算（可选）

### 5.3 编译选项
- C++17标准
- 启用RTTI（Ipopt需要）
- 优化选项：Release使用-O3，Debug使用-O0 -g

## 六、命名空间规范

建议使用统一的命名空间：`j6m_parking`

```cpp
namespace j6m_parking {
    // 公共类型
    namespace common { ... }
    
    // 配置管理
    namespace config { ... }
    
    // 数据映射
    namespace mapping { ... }
    
    // 功能模块
    namespace evaluation { ... }
    namespace recommendation { ... }
    // ...
}
```

## 七、模块间数据流

```
1. 外部感知数据 → Mapping → 内部数据结构
2. 内部数据 → 车位评估 → 有效车位列表
3. 有效车位 → 车位推荐 → 推荐车位（带评分）
4. 感知数据 → 栅格地图构建 → GridMap
5. 起始点 + 目标车位 + GridMap → Hybrid A* → 路径点序列
6. 路径点序列 → 轨迹平滑 → 平滑轨迹（按档位分段）
7. 每段轨迹 → 速度规划 → 带速度的轨迹段
8. 各段轨迹 → 轨迹合并 → 完整轨迹（位置+速度）
9. 完整轨迹 → 轨迹优化 → 优化后轨迹
10. 优化轨迹 → 控制轨迹输出 → 当前段轨迹（如3m内）
11. 控制轨迹 → Mapping → 外部控制数据格式
```

## 八、建议的实现顺序

1. **基础层**：common数据类型、Config模块、Mapping模块
2. **功能层1**：车位评估、车位推荐
3. **功能层2**：栅格地图构建、Hybrid A*
4. **功能层3**：轨迹平滑、速度规划、轨迹合并
5. **功能层4**：轨迹优化、控制轨迹输出

## 九、注意事项

1. **线程安全**：如果多线程使用，ConfigReader需要考虑线程安全
2. **内存管理**：统一使用智能指针，避免裸指针
3. **错误处理**：各模块应该提供清晰的错误码或异常机制
4. **日志系统**：建议集成日志系统（如spdlog），便于调试
5. **单元测试**：每个模块都应该有对应的单元测试
6. **文档**：接口应该有详细的Doxygen注释
