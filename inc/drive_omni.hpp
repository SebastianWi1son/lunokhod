#pragma once

#include "kinematics.hpp"

#include <cmath>

class OmniDrive : public Kinematics<OmniDrive> {
public:
    OmniDrive(uint8_t wn, float cr, float gamma, float wr)
        :wn_(wn), cr_(cr), gamma_(gamma), wr_(wr){}

    WheelSpeeds inverse_impl(const Twist& t_cmd) const {
        float J[6][3];
        for (uint8_t i = 0; i < wn_; ++i) {
            float th = (i * k2PI) / wn_ + gamma_;
            J[i][0] = -sinf(th)  / wr_;           // 见下面"数学实现"
            J[i][1] =  cosf(th)  / wr_;
            J[i][2] =  cr_ / wr_;
        }
        return jacobian_apply(J, wn_, t_cmd);
    }

    Twist forward_impl(const WheelSpeeds &ws_fb) const {
        Twist t;
        const float u0 = wr_ * ws_fb.values_[0];
        const float u1 = wr_ * ws_fb.values_[1];
        const float u2 = wr_ * ws_fb.values_[2];
        t.vx_ = (u2 - u1) / 1.7320508f;        // √3
        t.vy_ = (2.0f * u0 - u1 - u2) / 3.0f;
        t.wz_ = (u0 + u1 + u2) / (3.0f * cr_);
        return t;
    }

private:
    uint8_t wn_;
    float cr_;
    float gamma_;
    float wr_;
};