#pragma once

#include "contracts.hpp"
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"
#include <cstdint>
namespace lunokhod::chassis_loop {


struct ChassisLoopConfig {
    // --- Twist Acc limiter ---
    float acc_vx_ = 1.5f;
    float acc_vy_ = 1.5f;
    float acc_wz_ = 4.0f;
    // --- odometry ---
    float twist_scale_ = 1.0f;
};

template <typename ActuatorSet>
class ChassisLoop {
public:
    ChassisLoop(const ChassisLoopConfig& cfg, const ActuatorSet& actuators)
        : actuators_(actuators), limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_), odom_(cfg.twist_scale_) {}

    // ----- upstream interface -----
    void set_cmd(const Twist& cmd) { t_cmd_in_ = cmd; }
    // ----- odom record sink -----
    void set_sink(odometry::SampleSink sink, void* ctx = nullptr) { odom_.set_sink(sink, ctx); }
    // ----- heartbeat tick -----
    void tick(float dt, uint32_t now) {
        // --- limiter ---
        lim_res_ = limiter_.limit(t_cmd_in_, dt);
        t_cmd_final_ = lim_res_.out_;
        // --- inverse to ws ---
        ws_target_ = actuators_.inverse(t_cmd_final_);
        // --- push down + execute ---
        actuators_.apply(ws_target_, dt);
        // --- measure ---
        ws_meas_ = actuators_.measure(ws_target_);
        // --- forward to twist ---
        twist_meas_  = actuators_.forward(ws_meas_);
        // --- odom integral to pose ---
        odom_.update(twist_meas_, dt, now, t_cmd_final_, &ws_meas_);
    }

    // ----- getters -----
    Pose pose() const { return odom_.pose(); }
    float yaw_odo() const { return odom_.yaw_ref(); }
    Twist twist() const { return twist_meas_; }
    Twist cmd() const { return t_cmd_final_; }
    twist_acc_limiter::LimitResult limit_result() const { return lim_res_; }
    float wheel_speed(uint8_t i) const { return ws_meas_.values_[i]; }
    float wheel_target(uint8_t i) const { return ws_target_.values_[i]; }
    int16_t wheel_effort(uint8_t i) const { return actuators_.effort(i); }  // 最近一次 tick() 后该轮算出的 effort

private:
    ActuatorSet actuators_;
    twist_acc_limiter::TwistAccLimiter limiter_;
    twist_acc_limiter::LimitResult lim_res_{};       ///< 带 {}：默认初始化也要是"零值"，不是脏值
    odometry::Odometry odom_;

    Twist t_cmd_in_{ 0.0f, 0.0f, 0.0f };            ///< upstream cmd
    Twist t_cmd_final_{ 0.0f, 0.0f, 0.0f };         ///< actual twist push down
    WheelSpeeds ws_target_{};                                    ///< inverse result target
    WheelSpeeds ws_meas_{};
    Twist twist_meas_{ 0.0f, 0.0f, 0.0f };          ///< forward output
};

}  // namespace lunokhod::chassis_loop
