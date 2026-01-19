# 工程目录结构详细说明

## 完整目录树

```
J6M_Parking_PDC/
│
├── CMakeLists.txt                           # 主CMake构建文件
├── README.md                                # 项目说明文档
├── ARCHITECTURE.md                          # 架构设计文档
├── DIRECTORY_STRUCTURE.md                   # 目录结构说明（本文档）
│
├── cmake/                                   # CMake辅助模块
│   ├── FindIpopt.cmake                     # 查找Ipopt库
│   ├── FindOSQP.cmake                      # 查找OSQP库
│   └── CompilerOptions.cmake               # 编译器选项（可选）
│
├── include/                                 # 公共头文件（对外接口）
│   │
│   ├── common/                             # 公共数据类型定义
│   │   ├── types.h                        # GridMap, GridState等基础类型
│   │   ├── point.h                        # Point2D, Point3D
│   │   ├── trajectory.h                   # Trajectory, TrajectoryPoint
│   │   ├── parking_space.h                # ParkingSpace, ParkingSpaceType
│   │   └── obstacle.h                     # 障碍物相关类型（可选）
│   │
│   ├── config/                             # 配置管理模块接口
│   │   ├── config_reader.h                # ConfigReader类
│   │   └── config_structs.h               # 配置结构体定义（可选）
│   │
│   ├── mapping/                            # 数据映射层接口
│   │   └── data_mapper.h                  # DataMapper类
│   │
│   ├── parking_space_evaluation/          # 车位评估模块接口
│   │   └── parking_space_evaluator.h      # ParkingSpaceEvaluator类
│   │
│   ├── parking_space_recommendation/      # 车位推荐模块接口
│   │   └── parking_space_recommender.h    # ParkingSpaceRecommender类
│   │
│   ├── grid_map/                           # 栅格地图构建模块接口
│   │   └── grid_map_builder.h             # GridMapBuilder类
│   │
│   ├── hybrid_astar/                       # Hybrid A*算法模块接口
│   │   └── hybrid_astar_planner.h         # HybridAstarPlanner类
│   │
│   ├── trajectory_smoothing/               # 轨迹平滑模块接口
│   │   └── trajectory_smoother.h          # TrajectorySmoother类
│   │
│   ├── velocity_planning/                  # 速度规划模块接口
│   │   └── velocity_planner.h             # VelocityPlanner类
│   │
│   ├── trajectory_merging/                 # 轨迹合并模块接口
│   │   └── trajectory_merger.h            # TrajectoryMerger类
│   │
│   ├── trajectory_optimization/            # 轨迹优化模块接口
│   │   └── trajectory_optimizer.h         # TrajectoryOptimizer类
│   │
│   └── control_trajectory_output/          # 控制轨迹输出模块接口
│       └── control_trajectory_output.h     # ControlTrajectoryOutput类
│
├── src/                                     # 源代码实现目录
│   │
│   ├── common/                             # 公共类型实现
│   │   └── types.cpp                      # GridMap等方法实现
│   │
│   ├── config/                             # 配置管理实现
│   │   ├── config_reader.cpp              # ConfigReader实现
│   │   └── config_reader_impl.h           # 内部实现辅助（可选）
│   │
│   ├── mapping/                            # 数据映射层实现
│   │   ├── data_mapper.cpp                # DataMapper实现
│   │   └── internal_types.h               # 内部数据结构（如果需要）
│   │
│   ├── parking_space_evaluation/          # 车位评估实现
│   │   ├── parking_space_evaluator.cpp    # 评估器实现
│   │   └── parking_space_evaluator_impl.h # 内部辅助（可选）
│   │
│   ├── parking_space_recommendation/      # 车位推荐实现
│   │   ├── parking_space_recommender.cpp  # 推荐器实现
│   │   └── scoring_strategy.h             # 评分策略（可选）
│   │
│   ├── grid_map/                           # 栅格地图构建实现
│   │   ├── grid_map_builder.cpp           # 地图构建器实现
│   │   └── line_to_grid_converter.h       # 线段转栅格（可选）
│   │
│   ├── hybrid_astar/                       # Hybrid A*实现
│   │   ├── hybrid_astar_planner.cpp       # 规划器实现
│   │   ├── node.h                         # A*节点定义
│   │   └── heuristics.h                   # 启发式函数（可选）
│   │
│   ├── trajectory_smoothing/               # 轨迹平滑实现
│   │   ├── trajectory_smoother.cpp        # 平滑器实现
│   │   └── osqp_wrapper.h                 # OSQP封装（可选）
│   │
│   ├── velocity_planning/                  # 速度规划实现
│   │   ├── velocity_planner.cpp           # 速度规划器实现
│   │   └── velocity_profile.h             # 速度曲线类型（可选）
│   │
│   ├── trajectory_merging/                 # 轨迹合并实现
│   │   └── trajectory_merger.cpp          # 合并器实现
│   │
│   ├── trajectory_optimization/            # 轨迹优化实现
│   │   ├── trajectory_optimizer.cpp       # 优化器实现
│   │   └── ipopt_wrapper.h                # Ipopt封装（可选）
│   │
│   └── control_trajectory_output/          # 控制轨迹输出实现
│       └── control_trajectory_output.cpp   # 输出器实现
│
├── config/                                 # YAML配置文件目录
│   ├── common.yaml                         # 公共配置参数
│   ├── parking_space_evaluation.yaml       # 车位评估参数
│   ├── parking_space_recommendation.yaml   # 车位推荐参数
│   ├── grid_map.yaml                       # 栅格地图参数
│   ├── hybrid_astar.yaml                   # Hybrid A*参数
│   ├── trajectory_smoothing.yaml           # 轨迹平滑参数
│   ├── velocity_planning.yaml              # 速度规划参数
│   ├── trajectory_optimization.yaml        # 轨迹优化参数
│   └── control_trajectory_output.yaml      # 控制轨迹输出参数
│
├── test/                                   # 单元测试目录
│   ├── CMakeLists.txt                      # 测试CMake配置
│   ├── test_common/                        # 测试公共工具
│   │   └── test_utils.h                   # 测试辅助函数
│   ├── test_config/                        # 配置模块测试
│   │   └── test_config_reader.cpp         # ConfigReader测试
│   ├── test_mapping/                       # 映射层测试
│   │   └── test_data_mapper.cpp           # DataMapper测试
│   ├── test_parking_space_evaluation/      # 车位评估测试
│   │   └── test_parking_space_evaluator.cpp
│   ├── test_parking_space_recommendation/  # 车位推荐测试
│   │   └── test_parking_space_recommender.cpp
│   ├── test_grid_map/                      # 栅格地图测试
│   │   └── test_grid_map_builder.cpp
│   ├── test_hybrid_astar/                  # Hybrid A*测试
│   │   └── test_hybrid_astar_planner.cpp
│   └── ...                                 # 其他模块测试
│
├── examples/                               # 示例代码目录
│   ├── CMakeLists.txt                      # 示例CMake配置
│   ├── basic_usage.cpp                     # 基础使用示例
│   ├── full_pipeline.cpp                   # 完整流程示例
│   └── module_usage/                       # 各模块独立示例
│       ├── example_parking_evaluation.cpp  # 车位评估示例
│       ├── example_grid_map.cpp            # 栅格地图示例
│       ├── example_hybrid_astar.cpp        # Hybrid A*示例
│       └── ...                             # 其他模块示例
│
└── docs/                                   # 文档目录（可选）
    ├── API.md                              # API参考文档
    ├── MODULES.md                          # 模块详细说明
    └── CONFIG.md                           # 配置参数详细说明
```

## 目录说明

### 1. 根目录
- **CMakeLists.txt**: 主构建文件，配置编译选项、依赖库、子目录等
- **README.md**: 项目概述、编译说明、使用指南
- **ARCHITECTURE.md**: 架构设计文档，说明模块依赖关系
- **DIRECTORY_STRUCTURE.md**: 目录结构详细说明（本文档）

### 2. cmake/ 目录
存放CMake辅助模块，用于查找第三方库。

### 3. include/ 目录
**原则**：只包含对外接口的头文件，不包含实现细节。

- **common/**: 公共数据类型，所有模块都会使用
- **config/**: 配置管理接口
- **mapping/**: 数据映射接口
- **各功能模块目录**: 每个模块一个目录，包含该模块的对外接口头文件

### 4. src/ 目录
**原则**：实现代码放在这里，头文件中的接口通过包含src目录中对应实现来实现。

- **common/**: 公共类型的实现（如GridMap的方法实现）
- **各功能模块目录**: 每个模块的实现代码
- **可选的内部头文件**: 如 `*_impl.h`, `*_wrapper.h` 等，用于组织复杂实现

### 5. config/ 目录
存放YAML配置文件，每个模块对应一个配置文件，便于模块化管理和标定。

### 6. test/ 目录
单元测试代码，每个模块应有对应的测试文件。

### 7. examples/ 目录
示例代码，展示如何使用各个模块。

### 8. docs/ 目录（可选）
额外的文档，如API详细说明、模块使用指南等。

## 文件命名规范

### 头文件
- 类名使用大驼峰：`ConfigReader`, `HybridAstarPlanner`
- 头文件名使用小写下划线：`config_reader.h`, `hybrid_astar_planner.h`
- 对应关系：类名 → 文件名（转为小写下划线）

### 源文件
- 与头文件对应：`config_reader.h` → `config_reader.cpp`

### 配置文件
- YAML文件使用小写下划线：`parking_space_evaluation.yaml`
- 与模块名对应

### 测试文件
- 测试文件命名：`test_<module_name>.cpp`
- 例如：`test_config_reader.cpp`

## 模块组织原则

1. **一个模块一个目录**：每个功能模块在include和src下都有对应目录
2. **接口与实现分离**：接口在include，实现在src
3. **依赖最小化**：模块间依赖关系清晰，避免循环依赖
4. **配置独立**：每个模块的配置独立在config目录下
5. **便于测试**：每个模块都有对应的测试代码

## 与其他项目集成

### 作为静态库使用
编译后生成 `libj6m_parking_pdc.a`，其他项目可以：
1. 链接静态库
2. 包含 `include/` 目录下的头文件
3. 使用配置文件和Mapping层进行数据转换

### 库的导出
- 只导出 `include/` 目录下的头文件
- 不导出 `src/` 目录下的实现细节
- 配置文件可以通过安装规则复制到指定位置
