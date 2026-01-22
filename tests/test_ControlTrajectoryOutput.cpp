#include <gtest/gtest.h>
#include <iostream>

#include "ControlTrajectoryOutput.h"

using APS_Planning::ControlTrajectoryOutput;
using J6B_AD::APS_Planning::Planning_TrajectoryPoint;
using J6B_AD::APS_Planning::TrajectoryPoint;
using J6B_AD::APS_Planning::Pose;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;

// 构造一条简单的直线轨迹：沿 x 轴从 0 到 total_length，步长 step，GearPosition 固定
static std::vector<Planning_TrajectoryPoint> BuildStraightTrajectory(float total_length,
                                                                     float step,
                                                                     uint8_t gear) {
    std::vector<Planning_TrajectoryPoint> traj;
    int16_t s_mm = 0;
    int16_t t_ms = 0;
    int num_points = static_cast<int>(total_length / step) + 1;
    for (int i = 0; i < num_points; ++i) {
        float x = i * step;
        TrajectoryPoint tp{};
        tp.point.x = x;
        tp.point.y = 0.0f;
        tp.point.z = 0.0f;
        tp.heading = 0.0f;
        tp.kappa = 0.0f;
        tp.dkappa = 0.0f;
        tp.velocity = 1.0f;
        tp.acceleration = 0.0f;
        tp.jerk = 0.0f;
        tp.t = t_ms;
        t_ms += 100;
        tp.s = s_mm;
        s_mm = static_cast<int16_t>((i + 1) * step * 100.0f);  // 1m -> 100

        Planning_TrajectoryPoint pt{};
        pt.Trajectory = tp;
        pt.GearPosition = gear;
        traj.push_back(pt);
    }
    return traj;
}

// 情况 1：APS 未激活（APS_Parkingstate != 4）时，不应输出轨迹
TEST(ControlTrajectoryOutputTest, NoOutputWhenApsInactive) {
    ControlTrajectoryOutput output;

    auto traj = BuildStraightTrajectory(5.0f, 0.5f, static_cast<uint8_t>(3));

    Pose pose{};
    pose.position.x = 0.0f;
    pose.position.y = 0.0f;
    pose.position.z = 0.0f;
    pose.orientation.w = 1.0f;

    uint8_t APS_Parkingstate = 0;  // 非 4
    output.ControlTrajectoryOutputProcess(traj, pose, APS_Parkingstate);

    auto seg = output.GetCurrentSegTrojectory();
    auto seg_resampled = output.GetCurrentSegTrojectory_out();

    EXPECT_TRUE(seg.empty());
    EXPECT_TRUE(seg_resampled.empty());
}

// 情况 2：APS 激活，当前位置在轨迹起点附近，应截取前 3 米并重采样
TEST(ControlTrajectoryOutputTest, Front3mAndResampleWhenApsActive) {
    ControlTrajectoryOutput output;

    auto traj = BuildStraightTrajectory(10.0f, 0.5f, static_cast<uint8_t>(3));

    Pose pose{};
    pose.position.x = 0.0f;
    pose.position.y = 0.0f;
    pose.position.z = 0.0f;
    pose.orientation.w = 1.0f;

    uint8_t APS_Parkingstate = 4;  // APS active
    output.ControlTrajectoryOutputProcess(traj, pose, APS_Parkingstate);

    auto seg = output.GetCurrentSegTrojectory();
    auto seg_resampled = output.GetCurrentSegTrojectory_out();

    // 应该有非空截取段与重采样结果
    ASSERT_FALSE(seg.empty());
    ASSERT_FALSE(seg_resampled.empty());

    // 原始段应该从轨迹起点开始，长度大约 3m（允许一些插值误差）
    float first_x = seg.front().Trajectory.point.x;
    float last_x = seg.back().Trajectory.point.x;
    float length = last_x - first_x;
    EXPECT_NEAR(first_x, 0.0f, 1e-3f);
    EXPECT_NEAR(length, 3.0f, 0.3f);  // 允许 ±0.3m 误差

    // 重采样结果应该包含更多点，且 x 单调递增
    EXPECT_GE(seg_resampled.size(), seg.size());

    float prev_x = seg_resampled.front().Trajectory.point.x;
    for (const auto& p : seg_resampled) {
        EXPECT_GE(p.Trajectory.point.x, prev_x - 1e-4f);
        prev_x = p.Trajectory.point.x;
    }

    // 打印部分重采样结果，便于人工检查
    std::cout << "Resampled segment size = " << seg_resampled.size() << std::endl;
    std::cout << "index, x, y, s, gear" << std::endl;
    for (size_t i = 0; i < seg_resampled.size(); ++i) {
        const auto& tp = seg_resampled[i].Trajectory;
        uint8_t gear = seg_resampled[i].GearPosition;
        std::cout << i << ", "
                  << tp.point.x << ", "
                  << tp.point.y << ", "
                  << tp.s << ", "
                  << static_cast<int>(gear)
                  << std::endl;
    }
}

// 情况 3：空轨迹输入时，不应崩溃且输出为空
TEST(ControlTrajectoryOutputTest, EmptyTrajectoryInputProducesEmptyOutput) {
    ControlTrajectoryOutput output;

    std::vector<Planning_TrajectoryPoint> empty_traj;

    Pose pose{};
    pose.position.x = 0.0f;
    pose.position.y = 0.0f;
    pose.position.z = 0.0f;
    pose.orientation.w = 1.0f;

    uint8_t APS_Parkingstate = 4;
    output.ControlTrajectoryOutputProcess(empty_traj, pose, APS_Parkingstate);

    auto seg = output.GetCurrentSegTrojectory();
    auto seg_resampled = output.GetCurrentSegTrojectory_out();

    EXPECT_TRUE(seg.empty());
    EXPECT_TRUE(seg_resampled.empty());
}

// 情况 4：车辆已行驶一段距离后再次调用，截取起点应沿轨迹前移
TEST(ControlTrajectoryOutputTest, StartIndexMovesForwardAfterVehicleMoves) {
    ControlTrajectoryOutput output;

    // 10 米直线轨迹
    auto traj = BuildStraightTrajectory(10.0f, 0.5f, static_cast<uint8_t>(3));

    uint8_t APS_Parkingstate = 4;  // APS active

    // 第一次调用：车辆在 x=0
    Pose pose1{};
    pose1.position.x = 0.0f;
    pose1.position.y = 0.0f;
    pose1.position.z = 0.0f;
    pose1.orientation.w = 1.0f;

    output.ControlTrajectoryOutputProcess(traj, pose1, APS_Parkingstate);
    auto seg1 = output.GetCurrentSegTrojectory();
    ASSERT_FALSE(seg1.empty());
    float first_x1 = seg1.front().Trajectory.point.x;

    // 模拟车辆沿轨迹前进 2 米：多次调用以累积 Total_S
    Pose pose2{};
    pose2.position.y = 0.0f;
    pose2.position.z = 0.0f;
    pose2.orientation.w = 1.0f;

    // 这里分成几步前进，方便 Total_S 累积
    const int steps = 4;
    for (int i = 1; i <= steps; ++i) {
        float x = 0.5f * i;  // 每次 0.5m，最后到 x=2.0
        pose2.position.x = x;
        output.ControlTrajectoryOutputProcess(traj, pose2, APS_Parkingstate);
    }

    auto seg2 = output.GetCurrentSegTrojectory();
    ASSERT_FALSE(seg2.empty());
    float first_x2 = seg2.front().Trajectory.point.x;

    std::cout << "First segment start x1 = " << first_x1
              << ", start x2 = " << first_x2 << std::endl;

    // 期望第二次截取的起点比第一次更靠前（x 更大）
    EXPECT_GT(first_x2, first_x1 + 0.1f);
}


