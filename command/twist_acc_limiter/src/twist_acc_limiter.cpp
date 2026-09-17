#include "twist_acc_limiter.hpp"
namespace lunokhod::twist_acc_limiter {


TwistAccLimiter::TwistAccLimiter(float acc_vx, float acc_vy, float acc_wz)
        :acc_{ acc_vx, acc_vy, acc_wz }, prev_{ 0, 0, 0 } {}



LimitResult TwistAccLimiter::limit(const Twist& t_cmd, float dt) {       // time decoupling, need dt injected
    if (dt <= 0.0f) {       // against invalid dt injected
        prev_ = t_cmd;
        return {t_cmd, false, false, false};
    }
    LimitResult r;
    r.out_.vx_ = ramp(t_cmd.vx_, prev_.vx_, acc_.vx_, dt, r.is_vx_lim_);       // vx slew rate limit
    r.out_.vy_ = ramp(t_cmd.vy_, prev_.vy_, acc_.vy_, dt, r.is_vy_lim_);       // vy slew rate limit
    r.out_.wz_ = ramp(t_cmd.wz_, prev_.wz_, acc_.wz_, dt, r.is_wz_lim_);       // wz slew rate limit
    prev_ = r.out_;     // update state
    return r;
}

void TwistAccLimiter::reset() { prev_ = {0, 0, 0} ; }          // reset state



float TwistAccLimiter::ramp(float cmd, float prev, float acc, float dt, bool& is_lim) {
    float max_step = acc * dt;
    float lo = prev - max_step;
    float hi = prev + max_step;
    is_lim = (cmd < lo) || (cmd > hi);
    if (cmd < lo) { return lo; }
    if (cmd > hi) { return hi; }
    return cmd;
}

}  // namespace lunokhod::twist_acc_limiter
