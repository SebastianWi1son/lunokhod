#pragma once

#include "contracts.hpp"
#include <cmath>
#include <cstdint>

struct OdometrySample {
    uint32_t tick_;
    float dt_;
    WheelSpeeds ws_;
    Twist cmd_;
    Twist twist_;
    Pose pose_;
};

using SampleSink = void (*)(void* ctx, const OdometrySample&);

class Odometry {
public:
    explicit Odometry(float twist_scale = 1.0f) : x_(0.0f), y_(0.0f), yaw_(0.0f),
                        twist_scale_(twist_scale), sink_(nullptr), sink_ctx_(nullptr) {}

    void set_sink(SampleSink sink, void* ctx = nullptr) { sink_ = sink; sink_ctx_ = ctx; }

    void reset() { reset(Pose{0.0f, 0.0f, 0.0f}); }
    void reset(const Pose& p) { x_ = p.x_; y_ = p.y_; yaw_ = p.yaw_; }

    Pose update(const Twist& body_twist, float dt, uint32_t tick,
                const Twist& cmd = Twist{0.0f, 0.0f, 0.0f}, const WheelSpeeds *ws = nullptr) {
        // --- tri-channel scaling ---
        const float vx = twist_scale_ * body_twist.vx_;
        const float vy = twist_scale_ * body_twist.vy_;
        const float wz = twist_scale_ * body_twist.wz_;
        // 半隐式欧拉-先积分获取工作方向，再折算世界系位移
        yaw_ += wz * dt;
        const float c = std::cos(yaw_);
        const float s = std::sin(yaw_);
        x_ += (vx * c - vy * s) * dt;
        y_ += (vx * s + vy * c) * dt;
        // --- output ---
        const Pose p = pose();
        // --- Guard ---
        if (sink_ != nullptr && ws != nullptr) {
            const auto smp = OdometrySample{tick, dt, *ws, cmd, Twist{vx, vy, wz}, p};
            sink_(sink_ctx_, smp);
        }
        return p;
    }

    Pose pose() const { return Pose{x_, y_, wrap_(yaw_)}; }  // upper layer
    float yaw_continuous() const { return yaw_; }                      // cross layer
    float yaw_ref() const { return yaw_continuous(); }                 // cross layer verification output

private:
    // wrap to (-pi, pi]
    static float wrap_(float y) {
        const float pi = 3.14159265358979323846f;
        const float tp = 6.28318530717958647692f;

        float r = std::fmod(y + pi, tp);
        if (r < 0.0f) { r += tp; }
        return r - pi;
    }

    float x_;
    float y_;
    float yaw_;
    float twist_scale_;
    SampleSink sink_;
    void* sink_ctx_;
};