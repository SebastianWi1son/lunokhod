#pragma once

#include "pid.hpp"
#include "smooth_planner.hpp"
#include <cstdint>

namespace lunokhod::wheel {

using MeasureSpeedFn = float (*)(uint8_t motor_id, float dt);
using SetEffortFn    = void (*)(uint8_t motor_id, int16_t effort);   // ±1000 = PIDConfig::limit_out_

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
        SetEffortFn    set_effort);

    void set_cmd(float speed_cmd);
    void update(float dt);
    void stop();

    float   get_speed() const;
    int16_t effort()    const;      // 最近一次 update() 算出的 effort（= 传给 SetEffortFn 的值；没 update 过 = 0）
private:
    // --- Property ---
    uint8_t motor_id_;
    float speed_cmd_;
    float speed_cur_;
    int16_t effort_;                        // 最近一次 update() 算出的 effort（P26 单轮状态）
    // --- DSP Tools ---
    SmoothPlanner planner_;
    PID pid_;
    // --- Injective Func ---
    MeasureSpeedFn measure_speed_;
    SetEffortFn    set_effort_;
};

}  // namespace lunokhod::wheel
