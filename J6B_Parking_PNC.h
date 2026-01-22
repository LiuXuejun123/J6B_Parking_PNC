#ifndef J6B_PARKING_PLANNING_LIBRARY_H
#define J6B_PARKING_PLANNING_LIBRARY_H

#include <string>
#include <memory>
#include <vector>
#include <algorithm>


#include "APS_Planning_Datatype.h"
#include "Config_Reader.h"
#include "GridMapBuilder.h"
#include "ParkingSpaceEvaluator.h"
#include "ParkingSpaceRecommender.h"
#include "Hybrid_Astar.h"
#include "TrajectorySmoother.h"
#include "VelocityPlanner.h"
#include "TrajectoryMerger.h"
#include "TrajectoryOptimizer.h"
#include "ControlTrajectoryOutput.h"

using APS_Planning::common::Config_Reader;
using APS_ParkingSpace::ParkingSpaceEvaluator;
using APS_ParkingSpace::ParkingSpaceRecommender;
using APS_Planning::Hybrid_Astar;
using APS_Planning::ControlTrajectoryOutput;

using J6B_AD::APS_Planning::TrajectoryPoint;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;
using J6B_AD::APS_Planning::Pose;
using J6B_AD::APS_Planning::Planning_TrajectoryPoint;

namespace J6B_Parking_PNC {



    class J6BParkingPlanner {
    public:
        J6BParkingPlanner();
        ~J6BParkingPlanner();
    public:
        void parkingspacevalidateprocess(
            const J6B_AD::APS_Planning::APAFusionOutput &Current_APAFusionOutput,
            const J6B_AD::APS_Planning::LocationData &Current_LocationData,
            const J6B_AD::APS_Planning::ParkUIToPlanningData &Current_ParkUIToPlanningData,
            const J6B_AD::APS_Planning::ParkStateMachineData &Current_ParkStateMachineData,
            const J6B_AD::APS_Planning::UIToSTMData &Current_UIToSTMData,
            const J6B_AD::APS_Planning::BusToSTMData &Current_BusToSTMData,
            const J6B_AD::APS_Planning::VehicleConf &Current_VehicleConf,
            const J6B_AD::APS_Planning::FT_VehicleDataV3 &Current_FT_VehicleDataV3,
            const J6B_AD::APS_Planning::UIToPlanningDataDebug &Current_UIToPlanningDataDebug,
            J6B_AD::APS_Planning::ParkPlanningData &Current_ParkPlanningData,
            J6B_AD::APS_Planning::AlgInitSts &Current_AlgorithmInitSts,
            J6B_AD::APS_Planning::ParkPlaningState &Current_ParkPlaningState,
            J6B_AD::APS_Planning::ParkPlanningInfo &Current_ParkPlanningInfo,
            J6B_AD::APS_Planning::ParkPlanningUIInfo &Current_ParkPlanningUIInfo,
            J6B_AD::APS_Planning::ParkPlanningDebug &Current_ParkPlanningDebug);

    private:
        uint32_t sys_seq_;
        uint8_t planningState;
        // std::unique_ptr<Config_Reader> config_reader_;
        //std::unique_ptr<GridMapBuilder> grid_map_builder_;
        std::unique_ptr<ParkingSpaceEvaluator> parking_space_evaluator_;
        std::unique_ptr<ParkingSpaceRecommender> parking_space_recommender_;
        std::unique_ptr<Hybrid_Astar> hybrid_astar_;
        // std::unique_ptr<TrajectorySmoother> trajectory_smoother_;
        // std::unique_ptr<VelocityPlanner> velocity_planner_;
        // std::unique_ptr<TrajectoryMerger> trajectory_merger_;
        // std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer_;
        std::unique_ptr<ControlTrajectoryOutput> control_trajectory_output_;

        //
        std::vector<Planning_TrajectoryPoint> CurrentFullPath;





    };
}

#endif // J6B_PARKING_PLANNING_LIBRARY_H