#include <gtest/gtest.h>
#include "Planning_Math.h"

using namespace utils;

// ================
// GetQuadrilateralCenter 单元测试
// ================

TEST(PlanningMathTest, GetQuadrilateralCenter_SimpleRectangle) {
    Point3F p1{0.0f, 0.0f, 0.0f};
    Point3F p2{2.0f, 0.0f, 0.0f};
    Point3F p3{2.0f, 2.0f, 0.0f};
    Point3F p4{0.0f, 2.0f, 0.0f};

    Point3F center = GetQuadrilateralCenter(p1, p2, p3, p4);

    EXPECT_FLOAT_EQ(center.x, 1.0f);
    EXPECT_FLOAT_EQ(center.y, 1.0f);
    EXPECT_FLOAT_EQ(center.z, 0.0f);
}

TEST(PlanningMathTest, GetQuadrilateralCenter_NonZeroZ) {
    Point3F p1{0.0f, 0.0f, 1.0f};
    Point3F p2{0.0f, 0.0f, 3.0f};
    Point3F p3{0.0f, 0.0f, 5.0f};
    Point3F p4{0.0f, 0.0f, 7.0f};

    Point3F center = GetQuadrilateralCenter(p1, p2, p3, p4);

    // (1 + 3 + 5 + 7) / 4 = 4
    EXPECT_FLOAT_EQ(center.z, 4.0f);
}

// ================
// DistanceSquared 单元测试
// ================

TEST(PlanningMathTest, DistanceSquared_ZeroDistance) {
    Point3F a{1.0f, 2.0f, 0.0f};
    Point3F b{1.0f, 2.0f, 5.0f};  // z 不参与计算

    double d2 = DistanceSquared(a, b);
    EXPECT_DOUBLE_EQ(d2, 0.0);
}

TEST(PlanningMathTest, DistanceSquared_PositiveDistance) {
    Point3F a{0.0f, 0.0f, 0.0f};
    Point3F b{3.0f, 4.0f, 10.0f};  // 只算 xy -> 3^2 + 4^2 = 25

    double d2 = DistanceSquared(a, b);
    EXPECT_DOUBLE_EQ(d2, 25.0);
}

// ================
// convert_to_2d_pose 单元测试
// ================

TEST(PlanningMathTest, ConvertTo2DPose_PositionXYCopied) {
    Point3FWithCovariance pos{};
    pos.x = 10.0f;
    pos.y = -5.0f;
    pos.z = 123.0f;  // 会被忽略

    Quaternion4FWithCovariance q{};
    // 单位四元数 (w=1, x=y=z=0)，yaw = 0
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;

    Pose2D pose = convert_to_2d_pose(pos, q);

    EXPECT_FLOAT_EQ(pose.x, 10.0f);
    EXPECT_FLOAT_EQ(pose.y, -5.0f);
    EXPECT_FLOAT_EQ(pose.theta, 0.0f);
}

TEST(PlanningMathTest, ConvertTo2DPose_YawPiOver2) {
    Point3FWithCovariance pos{};
    pos.x = 0.0f;
    pos.y = 0.0f;
    pos.z = 0.0f;

    Quaternion4FWithCovariance q{};
    // 绕 z 轴旋转 90 度 (pi/2) 的四元数：
    // w = cos(theta/2), z = sin(theta/2)
    float theta = static_cast<float>(M_PI) / 2.0f;
    q.w = std::cos(theta / 2.0f);
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = std::sin(theta / 2.0f);

    Pose2D pose = convert_to_2d_pose(pos, q);

    EXPECT_NEAR(pose.theta, theta, 1e-5f);
}


