//
// 自动泊车规划系统使用示例
// 本示例展示如何编译成库后，在其他代码中调用各个模块
//

#include <iostream>
#include <string>

// 引入公共配置读取器
#include "common/config_reader/Config_Reader.h"

// 引入各个功能模块
#include "grid_map/GridMapBuilder.h"
#include "parking_space_evaluation/ParkingSpaceEvaluator.h"
#include "parking_sapce_recommendation/ParkingSpaceRecommender.h"
#include "hybrid_astar/Hybrid_Astar.h"
#include "trajectory_smoothing/TrajectorySmoother.h"
#include "velocity_planning/VelocityPlanner.h"
#include "trajectory_merging/TrajectoryMerger.h"
#include "trajectory_optimization/TrajectoryOptimizer.h"
#include "trajectory_output/ControlTrajectoryOutput.h"

using namespace APS_Planning;
using namespace APS_Planning::common;

/**
 * 示例：展示自动泊车规划系统的完整使用流程
 * 
 * 流程说明：
 * 1. 初始化配置读取器，加载配置文件
 * 2. 车位评估：对感知识别的车位进行有效性判定
 * 3. 车位推荐：从多个车位中选择最适合的车位
 * 4. 栅格地图构建：基于感知数据构建二维栅格地图
 * 5. 路径规划：使用Hybrid A*算法规划路径
 * 6. 轨迹平滑：对路径进行平滑处理
 * 7. 速度规划：为轨迹添加速度信息
 * 8. 轨迹合并：合并位置和速度信息
 * 9. 轨迹优化：进行非线性优化
 * 10. 控制轨迹输出：输出最终的控制轨迹
 */
int main(int argc, char* argv[]) {
    std::cout << "===========================================" << std::endl;
    std::cout << "自动泊车规划系统使用示例" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    // ============================================
    // 步骤1: 初始化配置读取器
    // ============================================
    std::cout << "\n[步骤1] 初始化配置读取器..." << std::endl;
    
    // 方式1: 使用默认配置文件路径
    // Config_Reader config_reader;
    
    // 方式2: 指定配置文件路径（推荐）
    std::string config_file = "../config/aps_planning_config.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }
    
    Config_Reader config_reader(config_file);
    std::cout << "配置文件路径: " << config_file << std::endl;
    
    // 读取各个模块的配置
    config_reader.read_grid_map_config();
    config_reader.read_parking_space_evaluation_config();
    config_reader.read_parking_space_recommendation_config();
    config_reader.read_trajectory_optimization_config();
    config_reader.read_trajectory_smoothing_config();
    config_reader.read_velocity_planning_config();
    config_reader.read_control_trajectory_output_config();
    
    // 获取配置示例
    auto grid_map_cfg = config_reader.get_grid_map_config();
    std::cout << "栅格地图分辨率: " << grid_map_cfg.resolution << " m" << std::endl;
    std::cout << "地图尺寸: " << grid_map_cfg.map_width << " x " 
              << grid_map_cfg.map_height << " m" << std::endl;
    
    // ============================================
    // 步骤2: 车位评估
    // ============================================
    std::cout << "\n[步骤2] 车位评估..." << std::endl;
    APS_ParkingSpace::ParkingSpaceEvaluator evaluator;
    // TODO: 调用评估接口
    // evaluator.evaluate(parking_space);
    std::cout << "车位评估完成" << std::endl;
    
    // ============================================
    // 步骤3: 车位推荐
    // ============================================
    std::cout << "\n[步骤3] 车位推荐..." << std::endl;
    // TODO: 创建车位推荐器实例
    // ParkingSpaceRecommender recommender;
    // auto recommended_space = recommender.recommend(parking_spaces);
    std::cout << "车位推荐完成" << std::endl;
    
    // ============================================
    // 步骤4: 栅格地图构建
    // ============================================
    std::cout << "\n[步骤4] 栅格地图构建..." << std::endl;
    APS_Planning::GridMapBuilder grid_map_builder;
    // TODO: 构建栅格地图
    // grid_map_builder.build(obstacles, freespace);
    std::cout << "栅格地图构建完成" << std::endl;
    
    // ============================================
    // 步骤5: Hybrid A*路径规划
    // ============================================
    std::cout << "\n[步骤5] Hybrid A*路径规划..." << std::endl;
    APS_Planning::Hybrid_Astar hybrid_astar;
    // TODO: 规划路径
    // auto path = hybrid_astar.plan(start_pose, goal_pose, grid_map);
    std::cout << "路径规划完成" << std::endl;
    
    // ============================================
    // 步骤6: 轨迹平滑
    // ============================================
    std::cout << "\n[步骤6] 轨迹平滑..." << std::endl;
    APS_Planning::TrajectorySmoother smoother;
    // TODO: 平滑轨迹
    // auto smoothed_trajectory = smoother.smooth(path);
    std::cout << "轨迹平滑完成" << std::endl;
    
    // ============================================
    // 步骤7: 速度规划
    // ============================================
    std::cout << "\n[步骤7] 速度规划..." << std::endl;
    APS_Planning::VelocityPlanner velocity_planner;
    // TODO: 规划速度
    // auto velocity_profile = velocity_planner.plan(smoothed_trajectory);
    std::cout << "速度规划完成" << std::endl;
    
    // ============================================
    // 步骤8: 轨迹合并
    // ============================================
    std::cout << "\n[步骤8] 轨迹合并..." << std::endl;
    APS_Planning::TrajectoryMerger merger;
    // TODO: 合并轨迹
    // auto merged_trajectory = merger.merge(smoothed_trajectory, velocity_profile);
    std::cout << "轨迹合并完成" << std::endl;
    
    // ============================================
    // 步骤9: 轨迹优化
    // ============================================
    std::cout << "\n[步骤9] 轨迹优化..." << std::endl;
    APS_Planning::TrajectoryOptimizer optimizer;
    // TODO: 优化轨迹
    // auto optimized_trajectory = optimizer.optimize(merged_trajectory);
    std::cout << "轨迹优化完成" << std::endl;
    
    // ============================================
    // 步骤10: 控制轨迹输出
    // ============================================
    std::cout << "\n[步骤10] 控制轨迹输出..." << std::endl;
    APS_Planning::ControlTrajectoryOutput output;
    // TODO: 输出控制轨迹
    // auto control_trajectory = output.get_control_trajectory(optimized_trajectory, current_pose);
    std::cout << "控制轨迹输出完成" << std::endl;
    
    std::cout << "\n===========================================" << std::endl;
    std::cout << "自动泊车规划流程执行完成！" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    return 0;
}
