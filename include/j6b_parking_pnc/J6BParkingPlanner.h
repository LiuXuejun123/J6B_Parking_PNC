//
// J6B Parking PNC 统一接口类
// 封装所有模块的调用，提供简洁的对外接口
//

#ifndef J6B_PARKING_PNC_PLANNER_H
#define J6B_PARKING_PNC_PLANNER_H

#include <string>
#include <memory>
#include <vector>
#include "common/datatype/APS_Planning_Datatype.h"
#include "common/config_reader/Config_Reader.h"
#include "grid_map/GridMapBuilder.h"
#include "parking_space_evaluation/ParkingSpaceEvaluator.h"
#include "parking_sapce_recommendation/ParkingSpaceRecommender.h"
#include "hybrid_astar/Hybrid_Astar.h"
#include "trajectory_smoothing/TrajectorySmoother.h"
#include "velocity_planning/VelocityPlanner.h"
#include "trajectory_merging/TrajectoryMerger.h"
#include "trajectory_optimization/TrajectoryOptimizer.h"
#include "trajectory_output/ControlTrajectoryOutput.h"

namespace J6B_Parking_PNC {
    class J6BParkingPlanner {
    public:
        J6BParkingPlanner();
        ~J6BParkingPlanner();
        void parkingspacevalidateprocess(
            const J6B_AD::APS_Planning::APAFusionOutput& Current_APAFusionOutput,
            const J6B_AD::APS_Planning::LocationData& Current_LocationData,
            const J6B_AD::APS_Planning::ParkUIToPlanningData& Current_ParkUIToPlanningData,
            const J6B_AD::APS_Planning::ParkStateMachineData& Current_ParkStateMachineData,
            const J6B_AD::APS_Planning::UIToSTMData& Current_UIToSTMData,
            const J6B_AD::APS_Planning::BusToSTMData&  Current_BusToSTMData,
            const J6B_AD::APS_Planning::VehicleConf& Current_VehicleConf,
            const J6B_AD::APS_Planning::FT_VehicleDataV3& Current_FT_VehicleDataV3,
            const J6B_AD::APS_Planning::UIToPlanningDataDebug& Current_UIToPlanningDataDebug,
            J6B_AD::APS_Planning::ParkPlanningData Current_ParkPlanningData,
            J6B_AD::APS_Planning::AlgInitSts Current_AlgorithmInitSts,
            J6B_AD::APS_Planning::ParkPlaningState Current_ParkPlaningState,
            J6B_AD::APS_Planning::ParkPlanningInfo Current_ParkPlanningInfo,
            J6B_AD::APS_Planning::ParkPlanningUIInfo Current_ParkPlanningUIInfo,
            J6B_AD::APS_Planning::ParkPlanningDebug Current_ParkPlanningDebug
            );    
        void parkingplanningprocess(const J6B_AD::APS_Planning::Point3F& StartPos,const J6B_AD::APS_Planning::Point3F& EndPos);
        
    private:
        // 
        std::unique_ptr<APS_Planning::common::Config_Reader> config_reader_;
        //std::unique_ptr<GridMapBuilder> grid_map_builder_;
        std::unique_ptr<APS_ParkingSpace::ParkingSpaceEvaluator> parking_space_evaluator_;
        std::unique_ptr<APS_ParkingSpace::ParkingSpaceRecommender> parking_space_recommender_;
        std::unique_ptr<APS_Planning::Hybrid_Astar> hybrid_astar_;
        // std::unique_ptr<TrajectorySmoother> trajectory_smoother_;
        // std::unique_ptr<VelocityPlanner> velocity_planner_;
        // std::unique_ptr<TrajectoryMerger> trajectory_merger_;
        // std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer_;
        std::unique_ptr<APS_Planning::ControlTrajectoryOutput> control_trajectory_output_;

        J6B_AD::APS_Planning::TrajectoryPoint PlanningTrajectory[100];
    };
}

#endif