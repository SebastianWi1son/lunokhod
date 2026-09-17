#pragma once

#include "contracts.hpp"  // Twist
namespace lunokhod::twist_acc_limiter {


// Twist acc limiter Result
struct LimitResult {
    // 默认成员初始化器（2026-09-17 补）：任何 `LimitResult x;` 都是**确定值**，
    // 不会因"忘了初始化"读到垃圾（铁律 §2.5 不静默吞错）。
    Twist out_{};                                  // limited output
    bool is_vx_lim_ = false;                       // is limited
    bool is_vy_lim_ = false;
    bool is_wz_lim_ = false;
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

}  // namespace lunokhod::twist_acc_limiter
