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
    private:
        float32_t CalculatePlaneDistance(const Point3F& p1, const Point3F& p2);
        std::vector<Planning_TrajectoryPoint> GetFront3MTrajectory(
                                                                    const std::vector<Planning_TrajectoryPoint>& full_trajectory,
                                                                    const Point3F& current_pos,
                                                                    float32_t target_distance = 3.0f);    

    private:
        bool isParkingComplete;

    };
} // APS_Planning

#endif //J6B_PARKING_PNC_CONTROLTRAJECTORYOUTPUT_H