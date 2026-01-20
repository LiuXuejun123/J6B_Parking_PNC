//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_HYBRID_ASTAR_H
#define J6B_PARKING_PNC_HYBRID_ASTAR_H
#include <vector>
#include <cmath>
#include <chrono>
#include <cstdint>   // uint8_t

#include "common/datatype/APS_Planning_Datatype.h"
#include "common/math/math.h"
using J6B_AD::APS_Planning::TrajectoryPoint;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;
using J6B_AD::APS_Planning::Pose;
using J6B_AD::APS_Planning::Planning_TrajectoryPoint;
namespace APS_Planning {


    class Hybrid_Astar {
        public:
        u_int8_t Hybrid_Astar_PlanningProcess(const Pose &pos,const u_int32_t& Planning_Slot_ID );
        inline std::vector<Planning_TrajectoryPoint> GetHybridAstarPlanningTrajectory();
        private:
        std::vector<Planning_TrajectoryPoint> generateTrajectory(float self_x, float self_y, float self_theta);
        std::vector<std::pair<float, float>> planVelocity(float total_dist, float max_vel, bool forward);
        std::vector<Planning_TrajectoryPoint> PlanningTrajectory;
        uint32_t Last_Planning_Slot_ID = uint32_t(0);// last time target id

    };
} // APS_Planning

#endif //J6B_PARKING_PNC_HYBRID_ASTAR_H