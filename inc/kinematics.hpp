#pragma once

#include "contracts.hpp"

constexpr float k2PI = 6.283185307179586f;

// ----- 行为层: CRTP Base, to upper chassis API, only assign -----
template<typename Derived>
class Kinematics {
public:
    WheelSpeeds inverse_kinematics(const Twist &cmd) const {          // const 不改变 this
        return static_cast<const Derived*>(this)->inverse_impl(cmd);
    }
    Twist forward_kinematics(const WheelSpeeds &fb) const {
        return static_cast<const Derived*>(this)->forward_impl(fb);
    }
};

// ----- 翻译官: J 矩阵 x 速度向量, pure matrix calc -----
template<uint8_t N>
WheelSpeeds jacobian_apply(const float (&J)[N][3], uint8_t wn, const Twist& t_cmd) {
    WheelSpeeds out_ws;
    out_ws.count_ = wn;
    for (uint8_t i = 0; i < N; ++i) {
        out_ws.values_[i] = J[i][0] * t_cmd.vx_  // Jacobian Calc
                          + J[i][1] * t_cmd.vy_
                          + J[i][2] * t_cmd.wz_;
    }
    return out_ws;
}

