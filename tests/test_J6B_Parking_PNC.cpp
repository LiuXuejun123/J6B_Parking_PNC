#include <gtest/gtest.h>
#include <iostream>

#include "../J6B_Parking_PNC.h"

using J6B_Parking_PNC::J6BParkingPlanner;

using J6B_AD::APS_Planning::APAFusionOutput;
using J6B_AD::APS_Planning::LocationData;
using J6B_AD::APS_Planning::ParkUIToPlanningData;
using J6B_AD::APS_Planning::ParkStateMachineData;
using J6B_AD::APS_Planning::UIToSTMData;
using J6B_AD::APS_Planning::BusToSTMData;
using J6B_AD::APS_Planning::VehicleConf;
using J6B_AD::APS_Planning::FT_VehicleDataV3;
using J6B_AD::APS_Planning::UIToPlanningDataDebug;
using J6B_AD::APS_Planning::ParkPlanningData;
using J6B_AD::APS_Planning::AlgInitSts;
using J6B_AD::APS_Planning::ParkPlaningState;
using J6B_AD::APS_Planning::ParkPlanningInfo;
using J6B_AD::APS_Planning::ParkPlanningUIInfo;
using J6B_AD::APS_Planning::ParkPlanningDebug;

// ================
// J6BParkingPlanner 集成流程测试
// ================

TEST(J6BParkingPlannerTest, EndToEndPlanningAndOutputs) {
    // 注意：此测试需要 config/aps_planning_config.yaml 配置文件存在
    // 如果测试失败并提示找不到配置文件，请确保项目根目录下有 config/aps_planning_config.yaml
    J6BParkingPlanner planner;

    // 构造一个空闲车位（id=1），OccType=0
    APAFusionOutput apa{};
    apa.pld.pldFusionResultVaildSize = 1;
    apa.pld.pldFusionResult[0].id = 1;
    apa.pld.pldFusionResult[0].OccType = 0;

    // 当前位置在原点，yaw=0
    LocationData loc{};
    loc.pose.position.x = 0.0f;
    loc.pose.position.y = 0.0f;
    loc.pose.position.z = 0.0f;
    loc.pose.orientation.w = 1.0f;

    // UI 选中了 id=1 这个车位
    ParkUIToPlanningData ui2plan{};
    ui2plan.selectedSlot = 1;

    // 状态机允许规划（apaSubSysSts 非 9/4）
    ParkStateMachineData sm{};
    sm.apaSubSysSts = 0;

    UIToSTMData ui2stm{};
    BusToSTMData bus2stm{};
    VehicleConf veh_conf{};
    FT_VehicleDataV3 veh_data{};
    UIToPlanningDataDebug debug{};

    ParkPlanningData park_plan_data{};
    AlgInitSts alg_init{};
    ParkPlaningState park_state{};
    ParkPlanningInfo park_info{};
    ParkPlanningUIInfo park_ui_info{};
    ParkPlanningDebug park_debug{};

    planner.parkingspacevalidateprocess(
        apa,
        loc,
        ui2plan,
        sm,
        ui2stm,
        bus2stm,
        veh_conf,
        veh_data,
        debug,
        park_plan_data,
        alg_init,
        park_state,
        park_info,
        park_ui_info,
        park_debug
    );

    // 基本检查：算法初始化状态和时间戳被填充
    EXPECT_EQ(alg_init.Alg_Index, static_cast<uint8_t>(5));
    EXPECT_EQ(alg_init.InitStatus, static_cast<uint8_t>(1));
    EXPECT_EQ(alg_init.taskflowID, static_cast<uint8_t>(1));

    // 规划输出应包含至少一个车位推荐
    EXPECT_GE(park_info.planningSlotInfoSize, static_cast<uint8_t>(1));

    // 轨迹输出：如果规划成功，trajectoryPointsValidSize>0
    std::cout << "trajectoryPointsValidSize = "
              << park_plan_data.trajectoryPointsValidSize << std::endl;

    // 不强制要求一定 >0（取决于 Hybrid_Astar 和 ControlTrajectoryOutput 的行为），
    // 但如果 >0，则打印前几个点
    if (park_plan_data.trajectoryPointsValidSize > 0) {
        size_t n = std::min<size_t>(park_plan_data.trajectoryPointsValidSize, 5);
        std::cout << "First " << n << " trajectory points:" << std::endl;
        for (size_t i = 0; i < n; ++i) {
            const auto &tp = park_plan_data.trajectoryPoints[i];
            std::cout << i << ": ("
                      << tp.point.x << ", "
                      << tp.point.y << "), heading="
                      << tp.heading << ", v="
                      << tp.velocity << std::endl;
        }
    }

    // ParkPlaningState：如果有推荐车位，则 smBackGrndSlotAvail 应为 2 或 0（激活中被清零）
    std::cout << "smBackGrndSlotAvail = "
              << static_cast<int>(park_state.smBackGrndSlotAvail) << std::endl;
}

