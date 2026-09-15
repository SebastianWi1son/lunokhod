#pragma once

#include "kinematics.hpp"

#include <cmath>

class OmniDrive : public Kinematics<OmniDrive> {
public:
    OmniDrive(uint8_t wn, float cr, float gamma, float wr)
        :wn_(wn), cr_(cr), gamma_(gamma), wr_(wr) {}

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
        // 通用 N 轮伪逆（对任意 N、任意 γ 成立；N=3,γ=0 时化简为旧特例公式）
        //   vx = (2/N)·Σ(-sin th_i)·u_i
        //   vy = (2/N)·Σ( cos th_i)·u_i
        //   wz = Σ u_i / (N·cr)
        Twist t;
        float sx = 0.0f, sy = 0.0f, sw = 0.0f;
        for (uint8_t i = 0; i < wn_; ++i) {
            float th = (i * k2PI) / wn_ + gamma_;
            float u  = wr_ * ws_fb.values_[i];
            sx += -sinf(th) * u;
            sy +=  cosf(th) * u;
            sw +=  u;
        }
        const float n = wn_;
        t.vx_ = 2.0f * sx / n;
        t.vy_ = 2.0f * sy / n;
        t.wz_ = sw / (n * cr_);
        return t;
    }

private:
    uint8_t wn_;
    float cr_;
    float gamma_;
    float wr_;
};