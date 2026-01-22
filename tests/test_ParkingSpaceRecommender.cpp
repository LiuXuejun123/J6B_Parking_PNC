#include <gtest/gtest.h>

#include "ParkingSpaceRecommender.h"

using APS_ParkingSpace::ParkingSpaceRecommender;
using J6B_AD::APS_Planning::SlotFusionResult;
using J6B_AD::APS_Planning::LocationData;
using J6B_AD::APS_Planning::PlanningSlotInfo;
using J6B_AD::APS_Planning::Point3F;

// ================
// ParkingSpaceRecommender::ParkingSpaceRecommendProcess 单元测试
// ================

// 情况 1：selectedid != 0 且 apaSubSysSts 为 9/4，且在有效车位列表中能找到该 id
// 预期：只返回一个对应 id 的 PlanningSlotInfo，recommendLevel=1
TEST(ParkingSpaceRecommenderTest, ReturnsSelectedSlotWhenValid) {
    std::vector<SlotFusionResult> validSlots(3);

    validSlots[0].id = 100;
    validSlots[1].id = 200;  // 将选择它
    validSlots[2].id = 300;

    LocationData loc{};  // 这里位置不会被用到（直接 return）

    uint32_t selectedId = 200;
    uint8_t apaSubSysSts = 9;  // 满足 (9 || 4) 条件

    ParkingSpaceRecommender recommender;
    recommender.ParkingSpaceRecommendProcess(validSlots, loc, selectedId, apaSubSysSts);

    auto result = recommender.Get_ValidPalnningSlotList();

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].preSelectSlotId, selectedId);
    EXPECT_FALSE(result[0].isNarrowSlot);
    EXPECT_EQ(result[0].recommendLevel, static_cast<uint8_t>(1));
}

// 情况 2：selectedid == 0 或者 apaSubSysSts 不为 9/4，按距离排序推荐
// 这里构造 3 个车位，假定车辆在原点，根据中心点距离排序
TEST(ParkingSpaceRecommenderTest, RanksSlotsByDistanceToLocation) {
    std::vector<SlotFusionResult> validSlots(3);

    // slot 1: 中心点在 (10,0)
    validSlots[0].id = 1;
    validSlots[0].entrancePointA = Point3F{10, 0, 0};
    validSlots[0].entrancePointB = Point3F{10, 0, 0};
    validSlots[0].tailPointC     = Point3F{10, 0, 0};
    validSlots[0].tailPointD     = Point3F{10, 0, 0};

    // slot 2: 中心点在 (3,0) -> 最近
    validSlots[1].id = 2;
    validSlots[1].entrancePointA = Point3F{3, 0, 0};
    validSlots[1].entrancePointB = Point3F{3, 0, 0};
    validSlots[1].tailPointC     = Point3F{3, 0, 0};
    validSlots[1].tailPointD     = Point3F{3, 0, 0};

    // slot 3: 中心点在 (6,0)
    validSlots[2].id = 3;
    validSlots[2].entrancePointA = Point3F{6, 0, 0};
    validSlots[2].entrancePointB = Point3F{6, 0, 0};
    validSlots[2].tailPointC     = Point3F{6, 0, 0};
    validSlots[2].tailPointD     = Point3F{6, 0, 0};

    LocationData loc{};
    loc.pose.position.x = 0.0f;
    loc.pose.position.y = 0.0f;
    loc.pose.position.z = 0.0f;

    uint32_t selectedId = 0;  // 不触发“直接选中”分支
    uint8_t apaSubSysSts = 0; // 任意不为 9/4 的值

    ParkingSpaceRecommender recommender;
    recommender.ParkingSpaceRecommendProcess(validSlots, loc, selectedId, apaSubSysSts);

    auto result = recommender.Get_ValidPalnningSlotList();

    ASSERT_EQ(result.size(), 3u);

    // 根据距离排序后，顺序应该是 id: 2 (3m) -> 3 (6m) -> 1 (10m)
    EXPECT_EQ(result[0].preSelectSlotId, 2u);
    EXPECT_EQ(result[1].preSelectSlotId, 3u);
    EXPECT_EQ(result[2].preSelectSlotId, 1u);

    // recommendLevel 应该从 1 递增
    EXPECT_EQ(result[0].recommendLevel, static_cast<uint8_t>(1));
    EXPECT_EQ(result[1].recommendLevel, static_cast<uint8_t>(2));
    EXPECT_EQ(result[2].recommendLevel, static_cast<uint8_t>(3));
}

// 情况 3：ValidSlot 为空时，结果应为空
TEST(ParkingSpaceRecommenderTest, EmptyValidSlotGivesEmptyResult) {
    std::vector<SlotFusionResult> validSlots;
    LocationData loc{};

    uint32_t selectedId = 0;
    uint8_t apaSubSysSts = 9;

    ParkingSpaceRecommender recommender;
    recommender.ParkingSpaceRecommendProcess(validSlots, loc, selectedId, apaSubSysSts);

    auto result = recommender.Get_ValidPalnningSlotList();
    EXPECT_TRUE(result.empty());
}

