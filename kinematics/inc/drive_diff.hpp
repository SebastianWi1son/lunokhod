#pragma once

#include "kinematics.hpp"
namespace lunokhod::kinematics {


// ----- 实现层: Drive Differential -----
class DiffDrive : public Kinematics<DiffDrive> {
public:
    DiffDrive(float wheelbase, float wheel_radius)
        : wb_(wheelbase), r_(wheel_radius) {}

    WheelSpeeds inverse_impl(const Twist &t_cmd) const {
        float J[2][3] = {
            { 1.0f / r_, 0.0f, -wb_ / (2.0f * r_) },   // Left
            { 1.0f / r_, 0.0f,  wb_ / (2.0f * r_) }    // Right
        };
        return jacobian_apply(J, 2, t_cmd);
    }

    Twist forward_impl(const WheelSpeeds &fb) const {
        Twist t;
        t.vx_ = r_ * (fb.values_[1] + fb.values_[0]) / 2.0f;
        t.wz_ = r_ * (fb.values_[1] - fb.values_[0]) / wb_;
        t.vy_ = 0;                                          // 非完整约束
        return t;
    }
// 只需要处理计算出ws和t
// 不需要持有该状态
private:
    float wb_;       // wheelbase (m)
    float r_;       // radius (m)
};

}  // namespace lunokhod::kinematics
