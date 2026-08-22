#pragma once

#include "contracts.hpp"  // Twist

// Twist acc limiter Result
struct LimitResult {
    Twist out_;                                    // limited output
    bool is_vx_lim_, is_vy_lim_, is_wz_lim_;       // is limited
};

class TwistAccLimiter {
public:
    TwistAccLimiter(float acc_vx, float acc_vy, float acc_wz);
    LimitResult limit(const Twist& t_cmd, float dt);
    void reset();          // reset state

private:
    static float ramp(float cmd, float prev, float acc, float dt, bool& is_lim);
    Twist acc_;
    Twist prev_;
};
