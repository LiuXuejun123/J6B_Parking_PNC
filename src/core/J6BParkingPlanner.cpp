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
#include <cmath>
#include <iostream>
#include <chrono>
#include <algorithm>
J6B_Parking_PNC::J6BParkingPlanner::J6BParkingPlanner()
{   
    // 初始化系统序列号
    this->sys_seq_ = 0;

    // 初始化配置读取器（可根据需要指定配置文件路径）
    this->config_reader_ = std::make_unique<APS_Planning::common::Config_Reader>();

    // 根据配置初始化各模块（后续可改为真正读取配置）
    // 目前 ParkingSpaceEvaluator / ParkingSpaceRecommender / Hybrid_Astar /
    // ControlTrajectoryOutput 都使用默认构造函数
    this->parking_space_evaluator_   = std::make_unique<APS_ParkingSpace::ParkingSpaceEvaluator>();
    this->parking_space_recommender_ = std::make_unique<APS_ParkingSpace::ParkingSpaceRecommender>();
    this->hybrid_astar_              = std::make_unique<APS_Planning::Hybrid_Astar>();
    this->control_trajectory_output_ = std::make_unique<APS_Planning::ControlTrajectoryOutput>();
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
    (void)Current_ParkPlanningDebug;
    (void)Current_UIToSTMData;
    (void)Current_BusToSTMData;
    (void)Current_VehicleConf;
    (void)Current_FT_VehicleDataV3;
    (void)Current_UIToPlanningDataDebug;

    J6B_AD::APS_Planning::TimeStamp NowTime;
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    // Time
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
    auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch - sec);
    NowTime.sec = static_cast<uint32_t>(sec.count());
    NowTime.nsec = static_cast<uint32_t>(ns.count());
    J6B_AD::APS_Planning::DataHeader NowHeader;

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
            this->planningState = PlanningState;
            if(PlanningState == uint8_t(1))
            {
                this->CurrentFullPath = this->hybrid_astar_->GetHybridAstarPlanningTrajectory();
            }
        }
        
    }
    // 
    // output process
    this->control_trajectory_output_-> ControlTrajectoryOutputProcess(this->hybrid_astar_->GetHybridAstarPlanningTrajectory(),Current_LocationData.pose,Current_ParkStateMachineData.apaSubSysSts);


    // 输出Current_ParkPlanningInfo

    Current_ParkPlanningInfo.header = NowHeader;
    size_t maxsize = std::min(this->parking_space_recommender_->Get_ValidPalnningSlotList().size(),size_t(20));
    for (size_t i = 0; i < maxsize;i++)
    {
        Current_ParkPlanningInfo.planningSlotInfo[i] = this->parking_space_recommender_->Get_ValidPalnningSlotList()[i];
    }

    Current_ParkPlanningInfo.planningSlotInfoSize = static_cast<uint8_t>(this->parking_space_recommender_->Get_ValidPalnningSlotList().size());


    //输出 ParkPlanningData
    Current_ParkPlanningData.header = NowHeader;
    Current_ParkPlanningData.targetPoint = this->hybrid_astar_->GetHybridAstarPlanningTrajectory().back().Trajectory.point;
    Current_ParkPlanningData.targetHeading = this->hybrid_astar_->GetHybridAstarPlanningTrajectory().back().Trajectory.heading;

    


    Current_ParkPlanningData.turnLight = 0;
    Current_ParkPlanningData.trajectoryPointsValidSize = this->control_trajectory_output_->GetCurrentSegTrojectory_out().size();
    for(size_t i = 0; i < this->control_trajectory_output_->GetCurrentSegTrojectory_out().size();i++)
    {
        Current_ParkPlanningData.trajectoryPoints[i] = this->control_trajectory_output_->GetCurrentSegTrojectory_out()[i].Trajectory;
    }

    Current_ParkPlanningData.gear = this->control_trajectory_output_->GetCurrentSegTrojectory().begin()->GearPosition;
    if(this->control_trajectory_output_->GetCurrentSegTrojectory_out().size() < 2)
    {
        Current_ParkPlanningData.gear = 0;//p
    }
   
    Current_ParkPlanningData.isSapa = 0;

    //输出 ParkPlanningState
    Current_ParkPlaningState.header = NowHeader;
    if (this->parking_space_recommender_->Get_ValidPalnningSlotList().size() >0)
    {
        Current_ParkPlaningState.smBackGrndSlotAvail = 2;
        if (Current_ParkStateMachineData.apaSubSysSts == u_int32_t(4))
        {
            Current_ParkPlaningState.smBackGrndSlotAvail = 0;
        }
        
    }

    if (this->planningState == 1)
    {
        Current_ParkPlaningState.smPathPlanning = 1;
         if (Current_ParkStateMachineData.apaSubSysSts == u_int32_t(4))
        {
            Current_ParkPlaningState.smBackGrndSlotAvail = 0;
        }
    }
    if(this->control_trajectory_output_->GetCurrentSegTrojectory_out().size() < 2)
    {
        Current_ParkPlaningState.smParkComplete = 1;//p
    }

    //Current_ParkPlanningUIInfo
    for(size_t i = 0; i < this->control_trajectory_output_->GetCurrentSegTrojectory_out().size();i++)
    {
        Current_ParkPlanningUIInfo.parkPathPoint[i].point = this->control_trajectory_output_->GetCurrentSegTrojectory_out()[i].Trajectory.point;
        Current_ParkPlanningUIInfo.parkPathPoint[i].heading = this->control_trajectory_output_->GetCurrentSegTrojectory_out()[i].Trajectory.heading;
    }

}
