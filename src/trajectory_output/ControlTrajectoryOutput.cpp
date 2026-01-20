//
// Created by lxj on 2026/1/19.
//

#include "ControlTrajectoryOutput.h"

namespace APS_Planning {

float32_t CalculatePlaneDistance(const Point3F& p1, const Point3F& p2) {
    float32_t dx = p1.x - p2.x;
    float32_t dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}
// ===================== 独立的插值函数 =====================
/**
 * @brief 对Point3F进行线性插值
 * @param a 起始点
 * @param b 终止点
 * @param ratio 插值比例（0~1）
 * @return 插值后的点
 */
Point3F LerpPoint3F(const Point3F& a, const Point3F& b, float32_t ratio) {
    Point3F result;
    result.x = a.x + (b.x - a.x) * ratio;
    result.y = a.y + (b.y - a.y) * ratio;
    result.z = a.z + (b.z - a.z) * ratio;
    return result;
}

/**
 * @brief 对TrajectoryPoint进行线性插值（独立函数）
 * @param a 起始轨迹点
 * @param b 终止轨迹点
 * @param ratio 插值比例（0~1）
 * @return 插值后的轨迹点
 */
TrajectoryPoint LerpTrajectoryPoint(const TrajectoryPoint& a, const TrajectoryPoint& b, float32_t ratio) {
    TrajectoryPoint result;
    // 位置插值
    result.point = LerpPoint3F(a.point, b.point, ratio);
    // 浮点属性线性插值
    result.heading = a.heading + (b.heading - a.heading) * ratio;
    result.kappa = a.kappa + (b.kappa - a.kappa) * ratio;
    result.dkappa = a.dkappa + (b.dkappa - a.dkappa) * ratio;
    result.velocity = a.velocity + (b.velocity - a.velocity) * ratio;
    result.acceleration = a.acceleration + (b.acceleration - a.acceleration) * ratio;
    result.jerk = a.jerk + (b.jerk - a.jerk) * ratio;
    // 整型属性插值后取整
    result.t = static_cast<int16_t>(a.t + (b.t - a.t) * ratio);
    result.s = static_cast<int16_t>(a.s + (b.s - a.s) * ratio);
    return result;
}

/**
 * @brief 对Planning_TrajectoryPoint进行线性插值（独立函数）
 * @param a 起始带档位轨迹点
 * @param b 终止带档位轨迹点
 * @param ratio 插值比例（0~1）
 * @return 插值后的带档位轨迹点
 * @throw runtime_error 若两个点档位不同
 */
Planning_TrajectoryPoint LerpPlanningTrajectoryPoint(const Planning_TrajectoryPoint& a, 
                                                     const Planning_TrajectoryPoint& b, 
                                                     float32_t ratio) {
    if (a.GearPosition != b.GearPosition) {
        return a;
    }
    Planning_TrajectoryPoint result;
    result.Trajectory = LerpTrajectoryPoint(a.Trajectory, b.Trajectory, ratio);
    result.GearPosition = a.GearPosition; // 档位保持不变
    return result;
}


/**
 * @brief 截取从当前位置开始的前3米轨迹（按档位分段）
 */
std::vector<Planning_TrajectoryPoint> GetFront3MTrajectory(
    const std::vector<Planning_TrajectoryPoint>& full_trajectory,
    const Point3F& current_pos,
    float32_t target_distance = 3.0f) {

    std::vector<Planning_TrajectoryPoint> result_trajectory;
    if (full_trajectory.empty()) {
        return result_trajectory;
    }

    // 1. 找到当前位置在轨迹中对应的起始索引（最接近current_pos的轨迹点）
    size_t start_index = 0;
    float32_t min_distance = CalculatePlaneDistance(full_trajectory[0].Trajectory.point, current_pos);
    for (size_t i = 1; i < full_trajectory.size(); ++i) {
        float32_t dist = CalculatePlaneDistance(full_trajectory[i].Trajectory.point, current_pos);
        if (dist < min_distance) {
            min_distance = dist;
            start_index = i;
        }
    }

    // 2. 获取当前档位，后续只处理同档位的轨迹段
    u_int8_t current_gear = full_trajectory[start_index].GearPosition;
    float32_t accumulated_distance = 0.0f;

    // 3. 从起始点开始累计距离，截取前3米（或当前档位剩余全部）
    for (size_t i = start_index; i < full_trajectory.size(); ++i) {
        // 档位变化则终止（只处理当前档位段）
        if (full_trajectory[i].GearPosition != current_gear) {
            break;
        }

        // 加入当前轨迹点到结果
        result_trajectory.push_back(full_trajectory[i]);

        // 计算当前点与下一个点的距离（最后一个点无后续，无需计算）
        if (i + 1 < full_trajectory.size() && full_trajectory[i+1].GearPosition == current_gear) {
            float32_t segment_dist = CalculatePlaneDistance(
                full_trajectory[i].Trajectory.point,
                full_trajectory[i+1].Trajectory.point);
            accumulated_distance += segment_dist;

            // 累计距离达到3米则终止
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
 * @param input_trajectory 输入轨迹（需保证同档位）
 * @param sample_interval 采样间距（米，默认0.05米=5厘米）
 * @return 重采样后的均匀轨迹
 */
std::vector<Planning_TrajectoryPoint> ResampleTrajectoryByDistance(
    const std::vector<Planning_TrajectoryPoint>& input_trajectory,
    float32_t sample_interval = 0.05f) {
    
    std::vector<Planning_TrajectoryPoint> resampled_trajectory;
    if (input_trajectory.size() < 2) {
        // 输入轨迹不足2个点，直接返回原轨迹
        return input_trajectory;
    }

    // 检查输入轨迹是否为同档位（重采样不跨档位）
    u_int8_t gear = input_trajectory[0].GearPosition;
    for (const auto& p : input_trajectory) {
        if (p.GearPosition != gear) {
            return resampled_trajectory;
        }
    }

    // 1. 计算输入轨迹的累计距离数组
    std::vector<float32_t> cumulative_distances;
    cumulative_distances.push_back(0.0f); // 第一个点距离为0
    float32_t total_distance = 0.0f;

    for (size_t i = 1; i < input_trajectory.size(); ++i) {
        float32_t seg_dist = CalculatePlaneDistance(input_trajectory[i-1].Trajectory.point,
                                                    input_trajectory[i].Trajectory.point);
        total_distance += seg_dist;
        cumulative_distances.push_back(total_distance);
    }

    // 2. 按固定间距生成采样点
    float32_t current_target_dist = 0.0f;
    size_t current_segment_index = 0; // 当前处理的轨迹段索引（i和i+1）

    // 先加入第一个点
    resampled_trajectory.push_back(input_trajectory[0]);

    while (current_target_dist < total_distance) {
        // 增加目标距离（5cm步长）
        current_target_dist += sample_interval;
        if (current_target_dist > total_distance) {
            current_target_dist = total_distance; // 最后一个点不超过总长度
        }

        // 找到目标距离所在的轨迹段
        while (current_segment_index < cumulative_distances.size() - 1 &&
               cumulative_distances[current_segment_index + 1] < current_target_dist) {
            current_segment_index++;
        }

        // 计算在当前段内的插值比例
        const auto& seg_start = input_trajectory[current_segment_index];
        const auto& seg_end = input_trajectory[current_segment_index + 1];
        float32_t seg_start_dist = cumulative_distances[current_segment_index];
        float32_t seg_end_dist = cumulative_distances[current_segment_index + 1];
        float32_t seg_length = seg_end_dist - seg_start_dist;

        float32_t ratio = 0.0f;
        if (seg_length > 1e-6) { // 避免除以0
            ratio = (current_target_dist - seg_start_dist) / seg_length;
        }

        // 调用独立的插值函数（核心修改点）
        Planning_TrajectoryPoint new_point = LerpPlanningTrajectoryPoint(seg_start, seg_end, ratio);
        resampled_trajectory.push_back(new_point);

        // 达到总长度则退出
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

        if(APS_Parkingstate == uint8_t(4))//aps active
        {
            std::vector<Planning_TrajectoryPoint> output_Trajectory = this->GetFront3MTrajectory(PlanningTrajectory,current_position,3.0);

        }

}

} // APS_Planning