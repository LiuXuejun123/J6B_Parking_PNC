# J6M 自动泊车规控系统 (Parking PDC)

## 项目概述

本项目实现了自动泊车规控系统的核心组件，采用C++17标准开发，使用CMake构建，可编译为静态链接库。

## 功能模块

### 1. 车位评估 (Parking Space Evaluation)
- 对感知识别的车位进行有效性判定
- 判定条件：宽度、长度、与自车运动方向的夹角等

### 2. 车位推荐 (Parking Space Recommendation)
- 维护自车周边的5-10个车位
- 设计推荐评分方案实现车位推荐

### 3. 栅格地图构建 (Grid Map Construction)
- 基于感知发送的车位位置、障碍物和Freespace线段
- 构建二维栅格地图

### 4. Hybrid_A*算法 (Hybrid A* Path Planning)
- 基于自车起始位置、目标车位位置和栅格地图
- 实现混合A*路径规划算法

### 5. 轨迹平滑 (Trajectory Smoothing)
- 按换档点拆分A*轨迹
- 分段优化：平滑度、均匀性、几何性
- 建立二次型优化，使用OSQP求解器

### 6. 速度规划 (Velocity Planning)
- 对单段轨迹进行速度规划
- 支持梯形图或正弦速度曲线

### 7. 轨迹合并 (Trajectory Merging)
- 将位置与速度合并后采样
- 生成完整轨迹

### 8. 轨迹优化 (Trajectory Optimization)
- 采用非线性优化方案
- 使用Ipopt实现非线性优化
- 约束：碰撞检测、曲率约束

### 9. 控制轨迹输出 (Control Trajectory Output)
- 系统泊车时，从规划后的轨迹中截取一段（如当前位置3m内）
- 外发控制轨迹

### 10. 配置管理 (Configuration Management)
- 使用YAML文件存储可标定参数
- ConfigRead类实现配置读取
- 各模块通过智能指针共享配置实例

### 11. 数据映射层 (Mapping Layer)
- 实现内部数据结构与外部数据结构的匹配
- 提供便捷的调用接口

## 工程架构

```
J6M_Parking_PDC/
├── CMakeLists.txt              # 主CMake构建文件
├── README.md                   # 项目说明文档
├── cmake/                      # CMake辅助模块
│   ├── FindOSQP.cmake         # OSQP库查找
│   └── FindIpopt.cmake        # Ipopt库查找
├── include/                    # 公共头文件目录
│   ├── common/                 # 公共数据类型定义
│   │   ├── types.h            # 基础数据类型
│   │   ├── point.h            # 点类型
│   │   ├── trajectory.h       # 轨迹类型
│   │   └── parking_space.h    # 车位类型
│   ├── config/                 # 配置管理
│   │   └── config_reader.h    # 配置读取器
│   └── mapping/                # 数据映射层
│       └── data_mapper.h      # 数据映射器
├── src/                        # 源代码目录
│   ├── config/                 # 配置管理实现
│   │   └── config_reader.cpp
│   ├── mapping/                # 数据映射实现
│   │   └── data_mapper.cpp
│   ├── parking_space_evaluation/  # 车位评估模块
│   │   ├── parking_space_evaluator.h
│   │   └── parking_space_evaluator.cpp
│   ├── parking_space_recommendation/ # 车位推荐模块
│   │   ├── parking_space_recommender.h
│   │   └── parking_space_recommender.cpp
│   ├── grid_map/               # 栅格地图构建
│   │   ├── grid_map_builder.h
│   │   └── grid_map_builder.cpp
│   ├── hybrid_astar/           # Hybrid_A*算法
│   │   ├── hybrid_astar_planner.h
│   │   └── hybrid_astar_planner.cpp
│   ├── trajectory_smoothing/   # 轨迹平滑
│   │   ├── trajectory_smoother.h
│   │   └── trajectory_smoother.cpp
│   ├── velocity_planning/      # 速度规划
│   │   ├── velocity_planner.h
│   │   └── velocity_planner.cpp
│   ├── trajectory_merging/     # 轨迹合并
│   │   ├── trajectory_merger.h
│   │   └── trajectory_merger.cpp
│   ├── trajectory_optimization/ # 轨迹优化
│   │   ├── trajectory_optimizer.h
│   │   └── trajectory_optimizer.cpp
│   └── control_trajectory_output/ # 控制轨迹输出
│       ├── control_trajectory_output.h
│       └── control_trajectory_output.cpp
├── config/                     # YAML配置文件目录
│   ├── parking_space_evaluation.yaml
│   ├── parking_space_recommendation.yaml
│   ├── grid_map.yaml
│   ├── hybrid_astar.yaml
│   ├── trajectory_smoothing.yaml
│   ├── velocity_planning.yaml
│   ├── trajectory_optimization.yaml
│   └── control_trajectory_output.yaml
├── test/                       # 测试代码目录
│   └── CMakeLists.txt
└── examples/                   # 示例代码目录
    └── CMakeLists.txt
```

## 依赖库

- **OSQP**: 二次型优化求解器（用于轨迹平滑）
- **Ipopt**: 非线性优化求解器（用于轨迹优化）
- **yaml-cpp**: YAML配置文件解析库
- **Eigen3**: 矩阵运算库（可选，推荐）

## 编译说明

```bash
mkdir build
cd build
cmake ..
make
```

## 使用说明

1. 配置参数：编辑 `config/` 目录下的YAML配置文件
2. 初始化配置：使用 `ConfigReader` 加载配置文件
3. 调用模块：通过 `DataMapper` 进行数据格式转换
4. 链接库：将编译生成的静态库链接到目标项目

## 许可证

[待补充]
