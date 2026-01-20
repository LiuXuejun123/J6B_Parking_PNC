//
// 自动泊车规划系统使用示例
// 本示例展示如何编译成库后，在其他代码中调用各个模块
//

#include <iostream>
#include <string>

// 方式1: 使用统一接口类（推荐，对外交付时使用）
#include "j6b_parking_pnc/J6BParkingPlanner.h"

// 方式2: 直接使用各个模块（高级用法）
// #include "common/config_reader/Config_Reader.h"
// #include "grid_map/GridMapBuilder.h"
// ...

using namespace APS_Planning;

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
    // 步骤1: 创建并初始化规划器
    // ============================================
    std::cout << "\n[步骤1] 初始化规划器..." << std::endl;
    
    J6BParkingPlanner planner;
    
    // 指定配置文件路径
    std::string config_file = "../config/aps_planning_config.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }
    
    if (!planner.Initialize(config_file)) {
        std::cerr << "Failed to initialize planner!" << std::endl;
        return -1;
    }
    
    std::cout << "配置文件路径: " << config_file << std::endl;
    std::cout << "规划器初始化成功" << std::endl;
    
    // ============================================
    // 方式1: 使用一键规划接口（推荐）
    // ============================================
    std::cout << "\n[方式1] 使用一键规划接口..." << std::endl;
    
    // 定义起始和目标位姿 [x, y, theta]
    double start_pose[3] = {0.0, 0.0, 0.0};
    double goal_pose[3] = {10.0, 5.0, 1.57};
    
    void* final_trajectory = nullptr; // TODO: 使用实际的数据结构
    void* obstacles = nullptr;         // TODO: 障碍物数据
    void* freespace = nullptr;       // TODO: 自由空间数据
    
    if (planner.PlanParkingTrajectory(start_pose, goal_pose, obstacles, freespace, final_trajectory)) {
        std::cout << "一键规划成功！" << std::endl;
    } else {
        std::cout << "一键规划失败" << std::endl;
    }
    
    // ============================================
    // 方式2: 分步骤调用各个模块
    // ============================================
    std::cout << "\n[方式2] 分步骤调用各个模块..." << std::endl;
    
    // 步骤2: 车位评估
    std::cout << "\n[步骤2] 车位评估..." << std::endl;
    void* parking_space_data = nullptr; // TODO: 车位数据
    bool is_valid = planner.EvaluateParkingSpace(parking_space_data);
    std::cout << "车位有效性: " << (is_valid ? "有效" : "无效") << std::endl;
    
    // 步骤3: 车位推荐
    std::cout << "\n[步骤3] 车位推荐..." << std::endl;
    void* parking_spaces = nullptr; // TODO: 车位列表
    auto recommended = planner.RecommendParkingSpaces(parking_spaces, 3);
    std::cout << "推荐了 " << recommended.size() << " 个车位" << std::endl;
    
    // 步骤4: 栅格地图构建
    std::cout << "\n[步骤4] 栅格地图构建..." << std::endl;
    if (planner.BuildGridMap(obstacles, freespace)) {
        std::cout << "栅格地图构建成功" << std::endl;
    }
    
    // 步骤5: 路径规划
    std::cout << "\n[步骤5] 路径规划..." << std::endl;
    void* path_output = nullptr;
    if (planner.PlanPath(start_pose, goal_pose, path_output)) {
        std::cout << "路径规划成功" << std::endl;
    }
    
    // 步骤6: 轨迹平滑
    std::cout << "\n[步骤6] 轨迹平滑..." << std::endl;
    void* smoothed_trajectory = nullptr;
    if (planner.SmoothTrajectory(path_output, smoothed_trajectory)) {
        std::cout << "轨迹平滑成功" << std::endl;
    }
    
    // 步骤7: 速度规划
    std::cout << "\n[步骤7] 速度规划..." << std::endl;
    void* velocity_profile = nullptr;
    if (planner.PlanVelocity(smoothed_trajectory, velocity_profile)) {
        std::cout << "速度规划成功" << std::endl;
    }
    
    // 步骤8: 轨迹合并
    std::cout << "\n[步骤8] 轨迹合并..." << std::endl;
    void* merged_trajectory = nullptr;
    if (planner.MergeTrajectory(smoothed_trajectory, velocity_profile, merged_trajectory)) {
        std::cout << "轨迹合并成功" << std::endl;
    }
    
    // 步骤9: 轨迹优化
    std::cout << "\n[步骤9] 轨迹优化..." << std::endl;
    void* optimized_trajectory = nullptr;
    if (planner.OptimizeTrajectory(merged_trajectory, optimized_trajectory)) {
        std::cout << "轨迹优化成功" << std::endl;
    }
    
    // 步骤10: 控制轨迹输出
    std::cout << "\n[步骤10] 控制轨迹输出..." << std::endl;
    double current_pose[3] = {1.0, 1.0, 0.5};
    void* control_trajectory = nullptr;
    if (planner.GetControlTrajectory(optimized_trajectory, current_pose, control_trajectory)) {
        std::cout << "控制轨迹输出成功" << std::endl;
    }
    
    // ============================================
    // 方式3: 直接访问底层模块（高级用法）
    // ============================================
    std::cout << "\n[方式3] 直接访问底层模块..." << std::endl;
    // 注意：直接访问底层模块需要包含相应的头文件
    // #include "common/config_reader/Config_Reader.h"
    // auto* hybrid_astar = planner.GetHybridAstar();
    // auto* smoother = planner.GetTrajectorySmoother();
    // auto* config_reader = planner.GetConfigReader();
    // if (config_reader) {
    //     auto grid_map_cfg = config_reader->get_grid_map_config();
    //     std::cout << "栅格地图分辨率: " << grid_map_cfg.resolution << " m" << std::endl;
    // }
    std::cout << "底层模块访问功能可用（需要包含相应头文件）" << std::endl;
    
    std::cout << "\n===========================================" << std::endl;
    std::cout << "自动泊车规划流程执行完成！" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    return 0;
}
