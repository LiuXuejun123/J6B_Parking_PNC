//
// Created by lxj on 2026/1/19.
//

#include "ControlTrajectoryOutput.h"
#include <limits>

namespace APS_Planning {

    /**
     * @brief 计算两个三维点在XY平面的欧氏距离（忽略Z轴）
     */
    float32_t ControlTrajectoryOutput::CalculatePlaneDistance(const Point3F& p1, const Point3F& p2) {
        float32_t dx = p1.x - p2.x;
        float32_t dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
 * @brief 对Point3F进行线性插值
 */
    Point3F ControlTrajectoryOutput::LerpPoint3F(const Point3F& a, const Point3F& b, float32_t ratio) {
        Point3F result;
        result.x = a.x + (b.x - a.x) * ratio;
        result.y = a.y + (b.y - a.y) * ratio;
        result.z = a.z + (b.z - a.z) * ratio;
        return result;
    }

    /**
     * @brief 对TrajectoryPoint进行线性插值
     */
    TrajectoryPoint ControlTrajectoryOutput::LerpTrajectoryPoint(const TrajectoryPoint& a, const TrajectoryPoint& b, float32_t ratio) {
        TrajectoryPoint result;
        result.point = LerpPoint3F(a.point, b.point, ratio);
        result.heading = a.heading + (b.heading - a.heading) * ratio;
        result.kappa = a.kappa + (b.kappa - a.kappa) * ratio;
        result.dkappa = a.dkappa + (b.dkappa - a.dkappa) * ratio;
        result.velocity = a.velocity + (b.velocity - a.velocity) * ratio;
        result.acceleration = a.acceleration + (b.acceleration - a.acceleration) * ratio;
        result.jerk = a.jerk + (b.jerk - a.jerk) * ratio;
        result.t = static_cast<int16_t>(a.t + (b.t - a.t) * ratio);
        result.s = static_cast<int16_t>(a.s + (b.s - a.s) * ratio);
        return result;
    }

    /**
     * @brief 对Planning_TrajectoryPoint进行线性插值
     */
    Planning_TrajectoryPoint ControlTrajectoryOutput::LerpPlanningTrajectoryPoint(const Planning_TrajectoryPoint& a,
                                                         const Planning_TrajectoryPoint& b,
                                                         float32_t ratio) {
        Planning_TrajectoryPoint result;
        // 若档位不同，这里简单采用起点的档位
        result.Trajectory = LerpTrajectoryPoint(a.Trajectory, b.Trajectory, ratio);
        result.GearPosition = a.GearPosition;
        return result;
    }


    /**
     * @brief 【核心封装函数】结合已行驶距离S和位置，精准查找轨迹起始索引
     * @param full_trajectory 完整轨迹列表
     * @param current_pos 当前后轴位置（Point3F）
     * @param traveled_S 车辆已行驶的累计距离（单位需与轨迹点s字段一致，如米）
     * @param s_weight S偏差的权重（默认0.7，位置距离权重0.3，可调整）
     * @return 最匹配的轨迹起始索引
     */
size_t ControlTrajectoryOutput::FindTrajectoryStartIndex(
    const std::vector<Planning_TrajectoryPoint>& full_trajectory,
    const Point3F& current_pos,
    float32_t traveled_S,
    float32_t s_weight ) {
    
    // 边界1：轨迹为空，返回0（或根据业务抛异常）
    if (full_trajectory.empty()) {
        
        return 0;
    }

    size_t best_index = 0;
    float32_t min_composite_score = std::numeric_limits<float32_t>::max(); // 综合评分（越小越优）

    // 1. 计算轨迹的S总范围（用于S偏差归一化）
    float32_t total_S_range = static_cast<float32_t>(
        full_trajectory.back().Trajectory.s - full_trajectory.front().Trajectory.s)/100;

    // 2. 计算轨迹的最大位置距离（用于位置距离归一化）
    float32_t max_pos_dist = 0.0f;
    const Point3F& first_point = full_trajectory[0].Trajectory.point;
    for (size_t j = 1; j < full_trajectory.size(); ++j) {
        float32_t dist = CalculatePlaneDistance(
            full_trajectory[j].Trajectory.point,
            first_point);
        max_pos_dist = std::max(max_pos_dist, dist);
    }

    // 遍历所有轨迹点，计算综合评分
    for (size_t i = 0; i < full_trajectory.size(); ++i) {
        const auto& traj_point = full_trajectory[i];
        float32_t point_S = static_cast<float32_t>(traj_point.Trajectory.s)/100; // 轨迹点的累计行驶距离

        // 1. 计算S偏差（归一化）
        float32_t s_diff = std::abs(point_S - traveled_S);
        float32_t normalized_s_diff = (total_S_range > 1e-6) ? (s_diff / total_S_range) : 0.0f;

        // 2. 计算位置距离（归一化）
        float32_t pos_dist = CalculatePlaneDistance(traj_point.Trajectory.point, current_pos);
        float32_t normalized_pos_dist = (max_pos_dist > 1e-6) ? (pos_dist / max_pos_dist) : 0.0f;

        // 3. 计算综合评分（S权重 + 位置权重）
        float32_t composite_score = (s_weight * normalized_s_diff) + ((1 - s_weight) * normalized_pos_dist);

        // 4. 更新最优索引
        if (composite_score < min_composite_score) {
            min_composite_score = composite_score;
            best_index = i;
        }
    }

    return best_index;
}




/**
 * @brief 截取从当前位置开始的前3米轨迹（按档位分段）
 */
std::vector<Planning_TrajectoryPoint> ControlTrajectoryOutput::GetFront3MTrajectory(
    const std::vector<Planning_TrajectoryPoint>& full_trajectory,
    const Point3F& current_pos,
    float32_t traveled_S, // 新增：已行驶距离S
    float32_t target_distance) {
    
    std::vector<Planning_TrajectoryPoint> result_trajectory;
    if (full_trajectory.empty()) {
        
        return result_trajectory;
    }

    // 调用封装函数，结合S和位置找起始索引
    size_t start_index = FindTrajectoryStartIndex(full_trajectory, current_pos, traveled_S);

    // 后续逻辑保持不变
    uint8_t current_gear = full_trajectory[start_index].GearPosition;
    float32_t accumulated_distance = 0.0f;

    for (size_t i = start_index; i < full_trajectory.size(); ++i) {
        if (full_trajectory[i].GearPosition != current_gear) {
            break;
        }

        result_trajectory.push_back(full_trajectory[i]);

        if (i + 1 < full_trajectory.size() && full_trajectory[i+1].GearPosition == current_gear) {
            float32_t segment_dist = CalculatePlaneDistance(
                full_trajectory[i].Trajectory.point,
                full_trajectory[i+1].Trajectory.point);
            accumulated_distance += segment_dist;

            if (accumulated_distance >= target_distance) {
                result_trajectory.push_back(full_trajectory[i+1]);
                break;
            }
        }
    }

    return result_trajectory;
}

/**
 * @brief 对轨迹进行重采样，生成固定间距的轨迹点（默认5cm）
 */
std::vector<Planning_TrajectoryPoint> ControlTrajectoryOutput::ResampleTrajectoryByDistance(
    const std::vector<Planning_TrajectoryPoint>& input_trajectory,
    float32_t sample_interval) {
    
    std::vector<Planning_TrajectoryPoint> resampled_trajectory;
    if (input_trajectory.size() < 2) {
        return input_trajectory;
    }

    uint8_t gear = input_trajectory[0].GearPosition;
    for (const auto& p : input_trajectory) {
        if (p.GearPosition != gear) {
           
        }
    }

    std::vector<float32_t> cumulative_distances;
    cumulative_distances.push_back(0.0f);
    float32_t total_distance = 0.0f;

    for (size_t i = 1; i < input_trajectory.size(); ++i) {
        float32_t seg_dist = CalculatePlaneDistance(input_trajectory[i-1].Trajectory.point,
                                                    input_trajectory[i].Trajectory.point);
        total_distance += seg_dist;
        cumulative_distances.push_back(total_distance);
    }

    float32_t current_target_dist = 0.0f;
    size_t current_segment_index = 0;

    resampled_trajectory.push_back(input_trajectory[0]);

    while (current_target_dist < total_distance) {
        current_target_dist += sample_interval;
        if (current_target_dist > total_distance) {
            current_target_dist = total_distance;
        }

        while (current_segment_index < cumulative_distances.size() - 1 &&
               cumulative_distances[current_segment_index + 1] < current_target_dist) {
            current_segment_index++;
        }

        const auto& seg_start = input_trajectory[current_segment_index];
        const auto& seg_end = input_trajectory[current_segment_index + 1];
        float32_t seg_start_dist = cumulative_distances[current_segment_index];
        float32_t seg_end_dist = cumulative_distances[current_segment_index + 1];
        float32_t seg_length = seg_end_dist - seg_start_dist;

        float32_t ratio = 0.0f;
        if (seg_length > 1e-6) {
            ratio = (current_target_dist - seg_start_dist) / seg_length;
        }

        Planning_TrajectoryPoint new_point = LerpPlanningTrajectoryPoint(seg_start, seg_end, ratio);
        resampled_trajectory.push_back(new_point);

        if (std::abs(current_target_dist - total_distance) < 1e-6) {
            break;
        }
    }

    return resampled_trajectory;
}



void ControlTrajectoryOutput::ControlTrajectoryOutputProcess(const std::vector<Planning_TrajectoryPoint> &PlanningTrajectory, const J6B_AD::APS_Planning::Pose &current_pos, uint8_t APS_Parkingstate)
{
        J6B_AD::APS_Planning::Point3F  current_position;
        current_position.x = current_pos.position.x;
        current_position.y = current_pos.position.y;
        current_position.z = current_pos.position.z;

        if (!initialized_) {
            // 首次调用：仅建立参考点，不累加里程
            this->Last_Position = current_position;
            this->Total_S = 0.0f;
            this->initialized_ = true;
        } else {
            this->Total_S += this->CalculatePlaneDistance(this->Last_Position,current_position);
            this->Last_Position = current_position;
        }

        if(APS_Parkingstate == 4)//aps active
        {
            // 截取前3米轨迹
            this->output_Trajectory = GetFront3MTrajectory(PlanningTrajectory, current_position,this->Total_S, 3.0f);
            this->output_Trajectory_Resampled = ResampleTrajectoryByDistance(this->output_Trajectory, 0.05f);

        }
}

  std::vector<Planning_TrajectoryPoint> ControlTrajectoryOutput::GetCurrentSegTrojectory()
{
    return this->output_Trajectory;
}

  std::vector<Planning_TrajectoryPoint> ControlTrajectoryOutput::GetCurrentSegTrojectory_out()
{
    return this->output_Trajectory_Resampled ;
}

} // APS_Planning