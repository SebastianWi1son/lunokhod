#pragma once

#include "lpf.hpp"
#include "ramp.hpp"

struct PIDConfig {
    float kp_ = 3.0f;
    float ki_ = 1.0f;
    float kd_ = 0.0f;

    float limit_out_ = 3600.0f;
    // --- i_term method property ---
    float limit_i_ = 1200.0f;
    float thresh_i_sep_ = 20.0f;
    // --- dsp tools property ---
    float max_rate_out_ = 10000.0f;
    float d_filter_Tf_ = 0.005f;
};

namespace pid_presets {
    inline constexpr PIDConfig wheel_speed;
}

class PID {
public:
    explicit PID(const PIDConfig &cfg);     // 显式确保PIDConfig作为参数参与构造
    float calc(float cmd, float measure, float dt);
    void reset();
private:
    // ----- Math Tools -----
    static float fabs(float val);
    static float constrainf(float val, float limit);

    // --- property ---
    PIDConfig cfg_;
    float integral_;
    float error_prev_;
    float measure_prev_;
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
};