//
// J6B Parking PNC 统一接口类实现
//

#include "J6BParkingPlanner.h"

// 包含各个模块的头文件
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

#include <iostream>
#include <chrono>

J6B_Parking_PNC::J6BParkingPlanner::J6BParkingPlanner()
{   
    
}

J6B_Parking_PNC::J6BParkingPlanner::~J6BParkingPlanner()
{

}

void J6B_Parking_PNC::J6BParkingPlanner::parkingspacevalidateprocess(
    const J6B_AD::APS_Planning::APAFusionOutput &Current_APAFusionOutput,
    const J6B_AD::APS_Planning::LocationData &Current_LocationData,
    const J6B_AD::APS_Planning::ParkUIToPlanningData &Current_ParkUIToPlanningData,
    const J6B_AD::APS_Planning::ParkStateMachineData &Current_ParkStateMachineData,
    const J6B_AD::APS_Planning::UIToSTMData &Current_UIToSTMData,
    const J6B_AD::APS_Planning::BusToSTMData &Current_BusToSTMData,
    const J6B_AD::APS_Planning::VehicleConf &Current_VehicleConf,
    const J6B_AD::APS_Planning::FT_VehicleDataV3 &Current_FT_VehicleDataV3,
    const J6B_AD::APS_Planning::UIToPlanningDataDebug &Current_UIToPlanningDataDebug,
    J6B_AD::APS_Planning::ParkPlanningData Current_ParkPlanningData,
    J6B_AD::APS_Planning::AlgInitSts Current_AlgorithmInitSts,
    J6B_AD::APS_Planning::ParkPlaningState Current_ParkPlaningState,
    J6B_AD::APS_Planning::ParkPlanningInfo Current_ParkPlanningInfo,
    J6B_AD::APS_Planning::ParkPlanningUIInfo Current_ParkPlanningUIInfo,
    J6B_AD::APS_Planning::ParkPlanningDebug Current_ParkPlanningDebug)
{
    J6B_AD::APS_Planning::TimeStamp NowTime;
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    // Time
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
    auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch - sec);
    NowTime.sec = static_cast<uint32_t>(sec.count());
    NowTime.nsec = static_cast<uint32_t>(ns.count());
    J6B_AD::APS_Planning:: DataHeader NowHeader;

    this->sys_seq_ ++;
    NowHeader.stamp = NowTime;
    NowHeader.seq = this->sys_seq_;

    // AlgInitSts
    Current_AlgorithmInitSts.Alg_Index = uint8_t(5);
    Current_AlgorithmInitSts.InitStatus = u_int8_t(1);
    Current_AlgorithmInitSts.taskflowID = u_int8_t(1);
    Current_AlgorithmInitSts.stamp = NowTime;

    // slot valid
    this->parking_space_evaluator_->ParkingSpaceEvaluate(Current_APAFusionOutput);
    this->parking_space_recommender_->ParkingSpaceRecommendProcess(this->parking_space_evaluator_->GetValidParkingSlot(),Current_LocationData,Current_ParkUIToPlanningData.selectedSlot,Current_ParkStateMachineData.apaSubSysSts);

    // planning
    if(Current_ParkUIToPlanningData.selectedSlot != u_int32_t(0))
    {
        if (Current_ParkStateMachineData.apaSubSysSts != u_int32_t(9) && Current_ParkStateMachineData.apaSubSysSts != u_int32_t(4))
        {
            uint8_t PlanningState =  this->hybrid_astar_->Hybrid_Astar_PlanningProcess(Current_LocationData.pose,Current_ParkUIToPlanningData.selectedSlot);
        }
        
    }

    //
    this->control_trajectory_output_-> 


}
