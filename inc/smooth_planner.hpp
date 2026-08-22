#pragma once

#include "ramp.hpp"
#include "lpf.hpp"

class SmoothPlanner {
public:
    SmoothPlanner(float max_rate, float Tf);
    float calc(float cmd, float dt);
    void reset();
private:
    Ramp ramp_;
    LPF f1_, f2_;
};