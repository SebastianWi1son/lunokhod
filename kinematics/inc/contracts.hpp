#pragma once

#include <cstdint>

// ----- Injection -----
struct Twist {
    float vx_;
    float vy_;
    float wz_;
};

// ----- Output -----
struct WheelSpeeds {
    uint8_t count_;         // 输出轮数
    float values_[6];       // 输出数据
};