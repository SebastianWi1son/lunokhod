#pragma once

#include "chassis.hpp"
#include "chassis_loop.hpp"
#include "contracts.hpp"
#include "wheel.hpp"
#include <array>
#include <cstdint>
namespace lunokhod::chassis_loop {


struct WheelSetConfig {
    wheel::PIDConfig           pid_{};        // 上游 ctlkit 的 PID 配置（三段式）
    wheel::SmoothPlannerConfig planner_{};
};

template <typename Chassis>
class WheelSet {
public:
    // --- constructor ---
    WheelSet(const WheelSetConfig& cfg, const Chassis& chassis,
             wheel::MeasureSpeedFn measure_speed, wheel::SetEffortFn set_effort)
        : chassis_(chassis),
          wheels_{
               {wheel::Wheel(0, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  wheel::Wheel(1, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  wheel::Wheel(2, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  wheel::Wheel(3, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  wheel::Wheel(4, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  wheel::Wheel(5, cfg.planner_, cfg.pid_, measure_speed, set_effort)}
          } {}

    // --- twist -> wheelspeeds ---
    WheelSpeeds inverse(const Twist& twist) const {
        WheelSpeeds sp = chassis_.inverse_kinematics(twist);
        for (uint8_t i = sp.count_; i < 6; ++i) { sp.values_[i] = 0.0f; }
        return sp;
    }

    // --- push down + execute ---
    void apply(const WheelSpeeds& sp, float dt) {
        const uint8_t wn = sp.count_;
        for (uint8_t i = 0; i < wn; ++i) {
            wheels_[i].set_cmd(sp.values_[i]);
            wheels_[i].update(dt);
        }
    }

    // --- get measure ----
    // 活跃数取自传入的设定值（与 apply 对称）→ 本类除轮子外无状态
    WheelSpeeds measure(const WheelSpeeds& sp) const {
        WheelSpeeds fb{};
        fb.count_ = sp.count_;
        for (uint8_t i = 0; i < sp.count_; ++i) { fb.values_[i] = wheels_[i].get_speed(); }
        return fb;
    }

    // --- per-wheel status ---
    // 最近一次 apply() 后该轮算出的 effort（只在前 count_ 个上有效；没 apply 过 = 0）
    int16_t effort(uint8_t i) const { return wheels_[i].effort(); }

    // --- wheelspeeds -> twist ---
    Twist forward(const WheelSpeeds& fb) const { return chassis_.forward_kinematics(fb); }

private:
    Chassis chassis_;
    std::array<wheel::Wheel, 6> wheels_;               ///< room for wheels
};

template <typename Chassis>
using WheelLoop = ChassisLoop<WheelSet<Chassis>>;

}  // namespace lunokhod::chassis_loop
