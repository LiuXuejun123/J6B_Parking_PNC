//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_MATH_H
#define J6B_PARKING_PNC_MATH_H

#include <cmath>
#include "APS_Planning_Datatype.h"


using J6B_AD::APS_Planning::Point3F;
using J6B_AD::APS_Planning::Point3FWithCovariance;
using J6B_AD::APS_Planning::Quaternion4FWithCovariance;

namespace utils {
    // 計算四邊形中心點（簡單平均）
    Point3F GetQuadrilateralCenter(
        Point3F p1, Point3F p2, Point3F p3, Point3F p4);

    // 兩點距離的平方（避免開平方根，常用於比較大小）
    double DistanceSquared(const Point3F& a, const Point3F& b);

    // 簡化的 2D 位姿結構（x, y, yaw）
    struct Pose2D {
        float x;
        float y;
        float theta;  // rad
        // 如果未來需要，可以在此加入協方差等欄位
    };

    // 從 3D 位置 + 四元數 轉換成 2D 位姿（只取 xy + yaw）
    Pose2D convert_to_2d_pose(
        const Point3FWithCovariance&      position,
        const Quaternion4FWithCovariance& orientation);
}

#endif //J6B_PARKING_PNC_MATH_H