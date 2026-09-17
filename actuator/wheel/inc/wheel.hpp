#pragma once

#include "pid.hpp"
#include "smooth_planner.hpp"
#include <cstdint>

using MeasureSpeedFn = float (*)(uint8_t motor_id, float dt);
using SetPwmFn       = void (*)(uint8_t motor_id, int16_t pwm);

// 规划器配置（SmoothPlanner 的参数聚合成块，默认 = legacy 实测值）
struct SmoothPlannerConfig {
    float max_rate_ = 1500.0f;   // 转速变化率上限（单位/秒，参考 rpm/s）
    float Tf_       = 0.02f;     // 滤波时间常数（秒）——与 LPF 的 Tf 同量纲
};

class Wheel {
public:
    Wheel(uint8_t motor_id,
        const SmoothPlannerConfig &sp_cfg,
        const PIDConfig &pid,
        MeasureSpeedFn measure_speed,
        SetPwmFn set_pwm);

    void set_cmd(float speed_cmd);
    void update(float dt);
    void stop();

    float get_speed() const;
private:
    // --- Property ---
    uint8_t motor_id_;
    float speed_cmd_;
    float speed_cur_;
    // --- DSP Tools ---
    SmoothPlanner planner_;
    PID pid_;
    // --- Injective Func ---
    MeasureSpeedFn measure_speed_;
    SetPwmFn set_pwm_;
};