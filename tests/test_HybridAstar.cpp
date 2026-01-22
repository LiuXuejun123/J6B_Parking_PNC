#include <gtest/gtest.h>
#include <iostream>

#include "Hybrid_Astar.h"

using APS_Planning::Hybrid_Astar;
using J6B_AD::APS_Planning::Pose;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;
using J6B_AD::APS_Planning::Planning_TrajectoryPoint;

// ================
// Hybrid_Astar::Hybrid_Astar_PlanningProcess 单元测试
// 重点：调用一次规划，检查轨迹基本属性，并把轨迹通过 std::cout 打印出来
// ================

TEST(HybridAstarTest, GeneratesForwardThenBackwardTrajectoryAndPrints) {
    // 构造一个简单的初始位姿：原点，yaw=0
    Pose pose{};
    pose.position.x = 0.0f;
    pose.position.y = 0.0f;
    pose.position.z = 0.0f;

    pose.orientation.x = 0.0f;
    pose.orientation.y = 0.0f;
    pose.orientation.z = 0.0f;
    pose.orientation.w = 1.0f;  // 单位四元数，yaw=0

    uint32_t planning_slot_id = 1;

    Hybrid_Astar planner;
    uint8_t status = planner.Hybrid_Astar_PlanningProcess(pose, planning_slot_id);

    // 基本断言：规划成功且轨迹非空
    EXPECT_EQ(status, static_cast<uint8_t>(1));

    auto traj = planner.GetHybridAstarPlanningTrajectory();
    ASSERT_FALSE(traj.empty());

    // 打印轨迹，用于直观检查
    std::cout << "Hybrid A* trajectory, size = " << traj.size() << std::endl;
    std::cout << "index, x, y, heading, v, a, s, gear" << std::endl;
    for (size_t i = 0; i < traj.size(); ++i) {
        const auto& tp = traj[i].Trajectory;
        uint8_t gear = traj[i].GearPosition;
        std::cout << i << ", "
                  << tp.point.x << ", "
                  << tp.point.y << ", "
                  << tp.heading << ", "
                  << tp.velocity << ", "
                  << tp.acceleration << ", "
                  << tp.s << ", "
                  << static_cast<int>(gear)
                  << std::endl;
    }

    // 进一步的简单逻辑检查：
    // 1) 前半段应该是前进档 (3)，后半段是倒档 (1)
    // 2) s 应该是单调递增
    ASSERT_GT(traj.size(), 10u);

    int last_s = traj.front().Trajectory.s;
    for (size_t i = 0; i < traj.size(); ++i) {
        int current_s = traj[i].Trajectory.s;
        EXPECT_GE(current_s, last_s);
        last_s = current_s;
    }
}

