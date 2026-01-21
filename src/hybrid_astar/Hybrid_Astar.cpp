//
// Created by lxj on 2026/1/19.
//

#include "Hybrid_Astar.h"

namespace APS_Planning {
    u_int8_t Hybrid_Astar::Hybrid_Astar_PlanningProcess(const Pose &pos,const u_int32_t& Planning_Slot_ID )
    {
        using Clock = std::chrono::steady_clock;
        using ms    = std::chrono::milliseconds;

        auto start_time = Clock::now();

        utils::Pose2D StartPos = utils::convert_to_2d_pose(pos.position, pos.orientation);
        this->Last_Planning_Slot_ID = Planning_Slot_ID;

        // 执行核心规划
        this->PlanningTrajectory = this->generateTrajectory(StartPos.x, StartPos.y, StartPos.theta);

        auto end_time = Clock::now();
        auto duration = std::chrono::duration_cast<ms>(end_time - start_time);

        constexpr ms TIMEOUT { 500 };  // 建议值：根据实际需求 300~800 ms

        if (duration > TIMEOUT) {
        // 可选：清空或标记无效轨迹
        this->PlanningTrajectory.clear();
        return 255;  // 超时
        }
        return 1;  // 成功

    }

      std::vector<Planning_TrajectoryPoint> Hybrid_Astar::GetHybridAstarPlanningTrajectory()
    {
        return this->PlanningTrajectory;
    }

    std::vector<Planning_TrajectoryPoint> Hybrid_Astar::generateTrajectory(float self_x, float self_y, float self_theta)
    {
          std::vector<Planning_TrajectoryPoint> trajectory;
    const float step = 0.1f;
    int timestamp = 0;
    float current_s = 0.0f;
    float forward_end_x = 0.0f; // 前进阶段的终点局部x坐标

    // --------------------- 第一阶段：前进5米 ---------------------
    float forward_dist = 6.0f;
    auto forward_vel_acc = planVelocity(forward_dist, 1.0f, true);
    for (size_t i = 0; i < forward_vel_acc.size(); i++) {
        TrajectoryPoint tp;
        float local_x = i * step;
        float local_y = 0.0f;

        tp.point.x = self_x + local_x * cos(self_theta) - local_y * sin(self_theta);
        tp.point.y = self_y + local_x * sin(self_theta) + local_y * cos(self_theta);
        tp.point.z = 0.0f;
        tp.heading = self_theta;
        tp.t = timestamp;
        timestamp += 100;
        tp.kappa = 0.0f;
        tp.dkappa = 0.0f;
        tp.velocity = forward_vel_acc[i].first;
        tp.acceleration = forward_vel_acc[i].second;
        tp.jerk = 0.0f;
        current_s += step;
        tp.s = static_cast<int16_t>(current_s * 100);

        Planning_TrajectoryPoint temp;
        temp.Trajectory = tp;
        temp.GearPosition = uint8_t(3);//D 
        trajectory.push_back(temp);

        // 记录前进阶段的终点局部x坐标
        if (i == forward_vel_acc.size() - 1) {
            forward_end_x = local_x;
        }
    }

    // --------------------- 第二阶段：倒退3米 ---------------------
    float backward_dist = 6.0f;
    auto backward_vel_acc = planVelocity(backward_dist, 0.8f, false);
    for (size_t i = 0; i < backward_vel_acc.size(); i++) {
        TrajectoryPoint tp;
        // 关键修复：从前进终点x开始，每步减少step，倒退3米
        float local_x = forward_end_x - (i + 1) * step;
        float local_y = 0.0f;

        tp.point.x = self_x + local_x * cos(self_theta) - local_y * sin(self_theta);
        tp.point.y = self_y + local_x * sin(self_theta) + local_y * cos(self_theta);
        tp.point.z = 0.0f;
        tp.heading = self_theta;
        tp.t = timestamp;
        timestamp += 100;
        tp.kappa = 0.0f;
        tp.dkappa = 0.0f;
        tp.velocity = backward_vel_acc[i].first;
        tp.acceleration = backward_vel_acc[i].second;
        tp.jerk = 0.0f;
        current_s += step;
        tp.s = static_cast<int16_t>(current_s * 100);
        Planning_TrajectoryPoint temp;
        temp.Trajectory = tp;
        temp.GearPosition = uint8_t(1);//R
        trajectory.push_back(temp);
    }

    return trajectory;
    }

    std::vector<std::pair<float, float>> Hybrid_Astar::planVelocity(float total_dist, float max_vel, bool forward)
    {
        const float step = 0.1f;
        int num_points = static_cast<int>(total_dist / step);
        std::vector<std::pair<float, float>> vel_acc(num_points);

        float accel = 1.0f;   // 加速度大小
        float accel_dist = max_vel * max_vel / (2 * accel);
        accel_dist = std::min(accel_dist, total_dist / 2);
        int accel_points = static_cast<int>(accel_dist / step);
        int decel_points = accel_points;
        int cruise_points = num_points - accel_points - decel_points;

        int idx = 0;
        float direction = forward ? 1.0f : -1.0f;

        // 加速阶段
        for (; idx < accel_points; idx++) {
            float vel = direction * accel * idx * step / max_vel * max_vel;
            vel_acc[idx] = {vel, direction * accel};
        }

        // 匀速阶段
        for (; idx < accel_points + cruise_points; idx++) {
            vel_acc[idx] = {direction * max_vel, 0.0f};
        }

        // 减速阶段
        for (; idx < num_points; idx++) {
            float decel_ratio = (idx - (accel_points + cruise_points)) * step / (max_vel * max_vel / (2 * accel));
            float vel = direction * max_vel * (1 - decel_ratio);
            vel_acc[idx] = {vel, -direction * accel};
        }

        return vel_acc;
    }

} // APS_Planning