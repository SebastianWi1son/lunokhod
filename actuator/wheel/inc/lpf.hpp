#pragma once

class LPF {
public:
    LPF(float Tf);
    float calc(float raw, float dt);
    void reset();
private:
    float Tf_;
    float prev_;
};