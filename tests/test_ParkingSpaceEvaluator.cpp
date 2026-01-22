#include <gtest/gtest.h>

#include "ParkingSpaceEvaluator.h"

using APS_ParkingSpace::ParkingSpaceEvaluator;
using J6B_AD::APS_Planning::APAFusionOutput;
using J6B_AD::APS_Planning::SlotFusionResult;

// ================
// ParkingSpaceEvaluator::ParkingSpaceEvaluate 单元测试
// ================

TEST(ParkingSpaceEvaluatorTest, FiltersOnlyOccTypeZeroSlots) {
    APAFusionOutput fusion{};

    // 构造 3 个 slot，其中 2 个 OccType=0，一个 OccType=1
    fusion.pld.pldFusionResultVaildSize = 3;

    fusion.pld.pldFusionResult[0].OccType = 0;  // 应被选中
    fusion.pld.pldFusionResult[0].id = 10;

    fusion.pld.pldFusionResult[1].OccType = 1;  // 不应被选中
    fusion.pld.pldFusionResult[1].id = 20;

    fusion.pld.pldFusionResult[2].OccType = 0;  // 应被选中
    fusion.pld.pldFusionResult[2].id = 30;

    ParkingSpaceEvaluator evaluator;
    evaluator.ParkingSpaceEvaluate(fusion);

    auto validSlots = evaluator.GetValidParkingSlot();

    ASSERT_EQ(validSlots.size(), 2u);
    EXPECT_EQ(validSlots[0].id, 10);
    EXPECT_EQ(validSlots[1].id, 30);
}

TEST(ParkingSpaceEvaluatorTest, NoValidSlotsWhenOccTypeNonZero) {
    APAFusionOutput fusion{};

    fusion.pld.pldFusionResultVaildSize = 2;
    fusion.pld.pldFusionResult[0].OccType = 1;
    fusion.pld.pldFusionResult[1].OccType = 2;

    ParkingSpaceEvaluator evaluator;
    evaluator.ParkingSpaceEvaluate(fusion);

    auto validSlots = evaluator.GetValidParkingSlot();
    EXPECT_TRUE(validSlots.empty());
}

