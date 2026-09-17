#pragma once

class Ramp {
public:
    Ramp(float max_rate);
    float calc(float cmd, float dt);
    void reset();
private:
    float max_rate_;
    float prev_;
};