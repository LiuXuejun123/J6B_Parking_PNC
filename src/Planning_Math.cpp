//
// Created by lxj on 2026/1/21.
//
//
// Created by lxj on 2026/1/19.
//

#include "Planning_Math.h"
#include <cmath>

namespace utils {

    Point3F GetQuadrilateralCenter(
        Point3F p1, Point3F p2, Point3F p3, Point3F p4)
    {
        Point3F center;
        center.x = (p1.x + p2.x + p3.x + p4.x) / 4.0f;
        center.y = (p1.y + p2.y + p3.y + p4.y) / 4.0f;
        center.z = (p1.z + p2.z + p3.z + p4.z) / 4.0f;
        return center;
    }

    double DistanceSquared(const Point3F& a, const Point3F& b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    Pose2D convert_to_2d_pose(
        const Point3FWithCovariance&      position,
        const Quaternion4FWithCovariance& orientation)
    {
        Pose2D p;

        // 位置直接取 xy
        p.x = position.x;
        p.y = position.y;
        // p.z 被忽略（停車規劃通常只關心 2D 平面）

        // 四元數轉 yaw（標準公式）
        float siny_cosp = 2.0f * (orientation.w * orientation.z +
                                  orientation.x * orientation.y);
        float cosy_cosp = 1.0f - 2.0f * (orientation.y * orientation.y +
                                         orientation.z * orientation.z);

        p.theta = std::atan2(siny_cosp, cosy_cosp);

        return p;
    }

}  // namespace utils