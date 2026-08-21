#pragma once

#include "kinematics.hpp"

class MecanumDrive : public Kinematics<MecanumDrive> {
public:
    MecanumDrive(float lx, float ly, float r)
        :lx_(lx), ly_(ly), r_(r) {}

    WheelSpeeds inverse_impl(const Twist& t_cmd) const {
        float J[4][3] = {
            { 1, -1, -(lx_ + ly_) },
            { 1,  1,  (lx_ + ly_) },
            { 1,  1, -(lx_ + ly_) },
            { 1, -1,  (lx_ + ly_) }
        };
        for (uint8_t i = 0; i < 4; ++i) {
            J[i][0] /= r_;
            J[i][1] /= r_;
            J[i][2] /= r_;
        }
        return jacobian_apply(J, 4, t_cmd);
    }

    Twist forward_impl(const WheelSpeeds& ws_fb) const {
        Twist t;
        t.vx_ = r_ * ( ws_fb.values_[0] + ws_fb.values_[1] + ws_fb.values_[2] + ws_fb.values_[3]) / 4.0f;         // ws_fb.values_[]具体哪个对哪个呢
        t.vy_ = r_ * (-ws_fb.values_[0] + ws_fb.values_[1] + ws_fb.values_[2] - ws_fb.values_[3]) / 4.0f;
        t.wz_ = r_ * (-ws_fb.values_[0] + ws_fb.values_[1] - ws_fb.values_[2] + ws_fb.values_[3]) / (4.0f * (lx_ + ly_));
        return t;
    }
private:
    float lx_;
    float ly_;
    float r_;
};


