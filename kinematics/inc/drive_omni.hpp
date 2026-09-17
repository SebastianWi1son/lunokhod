#pragma once

#include "kinematics.hpp"

#include <cmath>
namespace lunokhod::kinematics {


// 轮数 N 是**编译期事实**（模板参数，2026-09-17 定案）：
//   机械轮数"每台车一个、构建期就已知" → 做成运行期参数只会把"合法与否"拖到运行期
//   （历史上的越界写栈就是这么来的：契约容量 6 与运行期 wn 是两个互不认识的数）。
//   换成模板参数后：数组界 / 循环界 / 契约容量**天然一致**，非法轮数**写不出来**。
template <uint8_t N>
class OmniDrive : public Kinematics<OmniDrive<N>> {
public:
    static constexpr uint8_t kMaxWheels = 6;    // 上界 = WheelSpeeds::values_ 的容量（契约容量）
    // 下界 = 3：N=2（γ=0）时 J 的 vx 列全为零（-sin0 = -sinπ = 0）→ 秩只有 2，
    // vx 既发不出去（逆解恒为 0）也收不回来（正解恒为 0）——那不是全向底盘。
    // 实测秩：N=2 → 2，N=3/4/6 → 3（见 test/test_kinematics.cpp 的往返锚点）。
    static constexpr uint8_t kMinWheels = 3;

    static_assert(N >= kMinWheels && N <= kMaxWheels,
                  "OmniDrive: 轮数超出 [kMinWheels, kMaxWheels]"
                  "（下界 = 全向可解的最小值、上界 = WheelSpeeds 契约容量，见上方常量定义）");

    OmniDrive(float cr, float gamma, float wr) : cr_(cr), gamma_(gamma), wr_(wr) {}

    uint8_t wheel_count() const { return N; }   // 编译期就是 N

    WheelSpeeds inverse_impl(const Twist& t_cmd) const {
        float J[N][3];
        for (uint8_t i = 0; i < N; ++i) {
            float th = (i * k2PI) / N + gamma_;
            J[i][0] = -sinf(th)  / wr_;           // 见下面"数学实现"
            J[i][1] =  cosf(th)  / wr_;
            J[i][2] =  cr_ / wr_;
        }
        return jacobian_apply(J, N, t_cmd);
    }

    Twist forward_impl(const WheelSpeeds &ws_fb) const {
        // 通用 N 轮伪逆（对任意 N、任意 γ 成立；N=3,γ=0 时化简为旧特例公式）
        //   vx = (2/N)·Σ(-sin th_i)·u_i
        //   vy = (2/N)·Σ( cos th_i)·u_i
        //   wz = Σ u_i / (N·cr)
        Twist t;
        float sx = 0.0f, sy = 0.0f, sw = 0.0f;
        for (uint8_t i = 0; i < N; ++i) {
            float th = (i * k2PI) / N + gamma_;
            float u  = wr_ * ws_fb.values_[i];
            sx += -sinf(th) * u;
            sy +=  cosf(th) * u;
            sw +=  u;
        }
        const float n = N;
        t.vx_ = 2.0f * sx / n;
        t.vy_ = 2.0f * sy / n;
        t.wz_ = sw / (n * cr_);
        return t;
    }

private:
    float cr_;
    float gamma_;
    float wr_;
};



}  // namespace lunokhod::kinematics
