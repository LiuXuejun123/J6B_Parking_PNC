//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_CONTROLTRAJECTORYOUTPUT_H
#define J6B_PARKING_PNC_CONTROLTRAJECTORYOUTPUT_H
#include "common/datatype/APS_Planning_Datatype.h"
#include "common/math/math.h"
#include <cstdint>   // uint8_t
#include <vector>
#include <cmath>
using J6B_AD::APS_Planning::TrajectoryPoint;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;
using J6B_AD::APS_Planning::Pose;
using J6B_AD::APS_Planning::Planning_TrajectoryPoint;
typedef float float32_t;
typedef double float64_t;

namespace APS_Planning {
    class ControlTrajectoryOutput {
        public:
        void ControlTrajectoryOutputProcess(const std::vector<Planning_TrajectoryPoint>& PlanningTrajectory,
                                        const J6B_AD::APS_Planning::Pose& current_pos,
                                        uint8_t APS_Parkingstate);  
        
          std::vector<Planning_TrajectoryPoint> GetCurrentSegTrojectory();
          std::vector<Planning_TrajectoryPoint> GetCurrentSegTrojectory_out();
        private:
        float32_t Total_S;
        Point3F Last_Position;
        std::vector<Planning_TrajectoryPoint> output_Trajectory;
        std::vector<Planning_TrajectoryPoint> output_Trajectory_Resampled;
        
        private:
        float32_t CalculatePlaneDistance(const Point3F& p1, const Point3F& p2);
        Point3F LerpPoint3F(const Point3F& a, const Point3F& b, float32_t ratio);
        TrajectoryPoint LerpTrajectoryPoint(const TrajectoryPoint& a, const TrajectoryPoint& b, float32_t ratio);
        Planning_TrajectoryPoint LerpPlanningTrajectoryPoint(const Planning_TrajectoryPoint& a, const Planning_TrajectoryPoint& b, float32_t ratio);
        std::vector<Planning_TrajectoryPoint> GetFront3MTrajectory(
            const std::vector<Planning_TrajectoryPoint>& full_trajectory,
            const Point3F& current_pos,
            float32_t traveled_S, // 新增：已行驶距离S
            float32_t target_distance = 3.0f); 
        std::vector<Planning_TrajectoryPoint> ResampleTrajectoryByDistance(
            const std::vector<Planning_TrajectoryPoint>& input_trajectory,
            float32_t sample_interval = 0.05f);

    };
} // APS_Planning

#endif //J6B_PARKING_PNC_CONTROLTRAJECTORYOUTPUT_H