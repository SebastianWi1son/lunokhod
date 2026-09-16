#pragma once

#include "chassis.hpp"
#include "contracts.hpp"
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"
#include "wheel.hpp"
#include <cstdint>

struct ChassisLoopConfig {
    // --- Twist Acc limiter ---
    float acc_vx_ = 1.5f;
    float acc_vy_ = 1.5f;
    float acc_wz_ = 4.0f;
    // --- Wheel Control ---
    PIDConfig pid_{};
    SmoothPlannerConfig planner_{};
    // --- Odometry ---
    float twist_scale_ = 1.0f;
};

template <typename Chassis>
class ChassisLoop {
public:
    using MeasureSpeedFn = float (*)(uint8_t wheel_id, float dt);
    using SetPwmFn = void (*)(uint8_t wheel_id, int16_t pwm);

    // --- external chassis injection ---
    ChassisLoop(const ChassisLoopConfig& cfg, const Chassis& chassis, MeasureSpeedFn measure_speed, SetPwmFn set_pwm)
        : chassis_(chassis), limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_), odom_(cfg.twist_scale_),
          w0_(0, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          w1_(1, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          w2_(2, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          w3_(3, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          wheels_{ &w0_, &w1_, &w2_, &w3_ } {}

    // ----- upstream interface -----
    void set_cmd(const Twist& cmd) { t_cmd_in_ = cmd; }
    // ----- odom record sink -----
    void set_sink(SampleSink sink, void* ctx = nullptr) { odom_.set_sink(sink, ctx); }
    // ----- heartbeat tick -----
    void tick(float dt, uint32_t now) {
        // --- limiter ---
        t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_;
        // --- inverse to ws ---
        ws_target_ = chassis_.inverse_kinematics(t_cmd_final_);
        const uint8_t wn = ws_target_.count_;
        for (uint8_t i = wn; i < 6; ++i) { ws_target_.values_[i] = 0.0f; }
        // --- push down & actuator ---
        for (uint8_t i = 0; i < wn; ++i) {
            wheels_[i]->set_cmd(ws_target_.values_[i]);
            wheels_[i]->update(dt);
        }
        // --- measure ---
        WheelSpeeds meas{};
        meas.count_ = wn;
        for (uint8_t i = 0; i < wn; ++i) { meas.values_[i] = wheels_[i]->get_speed(); }
        // --- forward to twist ---
        twist_meas_  = chassis_.forward_kinematics(meas);
        // --- odom integral to pose ---
        // --- process to res残差 ---
        odom_.update(twist_meas_, dt, now, t_cmd_final_, &meas);
    }

    // ----- getters -----
    Pose pose() const { return odom_.pose(); }
    float yaw_odo() const { return odom_.yaw_ref(); }
    Twist twist() const { return twist_meas_; }
    Twist cmd() const { return t_cmd_final_; }
    float wheel_speed(uint8_t i) const { return wheels_[i]->get_speed(); }
    float wheel_target(uint8_t i) const { return ws_target_.values_[i]; }

private:
    Chassis chassis_;
    TwistAccLimiter limiter_;
    Odometry odom_;

    Wheel w0_, w1_, w2_, w3_;
    Wheel* wheels_[4];

    Twist t_cmd_in_{ 0.0f, 0.0f, 0.0f };            ///< upstream cmd
    Twist t_cmd_final_{ 0.0f, 0.0f, 0.0f };         ///< actual twist push down
    WheelSpeeds ws_target_{};                                    ///< inverse result target
    Twist twist_meas_{ 0.0f, 0.0f, 0.0f };          ///< forward output
};