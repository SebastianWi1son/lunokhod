#pragma once

#include "lpf.hpp"
#include "ramp.hpp"

struct PIDConfig {
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;

    float limit_out_ = 0.0f;
    // --- i_term method property ---
    float limit_i_ = 0.0f;
    float thresh_i_sep_ = 0.0f;
    // --- dsp tools property ---
    float max_rate_out_ = 0.0f;
    float d_filter_Tf_ = 0.0f;
};

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