//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_MATH_H
#define J6B_PARKING_PNC_MATH_H
#include "../datatype/APS_Planning_Datatype.h"
using J6B_AD::APS_Planning::Point3F;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;

namespace utils{
    Point3F GetQuadrilateralCenter(Point3F p1, Point3F p2, Point3F p3, Point3F p4) {
        Point3F center;
        center.x = (p1.x + p2.x + p3.x + p4.x) / 4.0;
        center.y = (p1.y + p2.y + p3.y + p4.y) / 4.0;
        center.z = (p1.z + p2.z + p3.z + p4.z) / 4.0;
        return center;
    }
    double DistanceSquared(const Point3F& a, const Point3F& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    struct Pose2D {
    float x;
    float y;
    float theta;          // rad
    // 如果需要协方差，可以扩展
    };

    Pose2D convert_to_2d_pose(
        const Point3FWithCovariance&      position,
        const Quaternion4FWithCovariance& orientation)
    {
        Pose2D p;

        // 位置直接取 xy
        p.x = position.x;
        p.y = position.y;
        // p.z 被忽略（或检查是否接近0）

        // 提取 yaw（最常用公式）
        float siny_cosp = 2.0f * (orientation.w * orientation.z +
                                orientation.x * orientation.y);
        float cosy_cosp = 1.0f - 2.0f * (orientation.y * orientation.y +
                                        orientation.z * orientation.z);

        p.theta = std::atan2(siny_cosp, cosy_cosp);

        return p;
    }

}



#endif //J6B_PARKING_PNC_MATH_H