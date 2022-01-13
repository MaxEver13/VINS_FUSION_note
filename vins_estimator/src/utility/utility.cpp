/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-12-01 16:02:49
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-12-17 10:16:41
 */
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    // 三个轴的平均加速度
    Eigen::Vector3d ng1 = g.normalized();
    // 世界坐标系为ENU坐标系，ng2即z轴--重力加速度方向的反方向
    Eigen::Vector3d ng2{0, 0, 1.0};
    // R0为ng1旋转到ng2,由于ng2只有z轴方向，所以这就将ng1与世界坐标系的z轴对齐了
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    // 由于旋转对齐z轴过程，产生了偏航角yaw，这里需要把yaw设为0.这样才保证跟世界坐标系x,y轴对齐
    // 最后这个R0才是真正的imu坐标系到世界坐标系的姿态
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
