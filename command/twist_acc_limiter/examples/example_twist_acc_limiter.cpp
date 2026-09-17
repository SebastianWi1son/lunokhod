// 链式演示：指令阶跃 → TwistAccLimiter 平滑 → DiffDrive 逆解 → 轮速
// 展示：限幅器把阶跃指令变成平滑斜坡，轮速单调爬升无跳变
#include "twist_acc_limiter.hpp"
#include "chassis.hpp"
#include <cstdio>

int main() {
    TwistAccLimiter limiter(1.0f, 1.0f, 2.0f);   // vx/vy: 1 m/s², wz: 2 rad/s²
    DiffDrive chassis(0.16f, 0.03f);             // 轮距 0.16m，轮半径 0.03m

    const float dt = 0.01f;                      // 10ms 控制周期
    const Twist cmd{0.5f, 0.3f, 1.0f};           // 指令阶跃

    printf("帧   限幅输出 (vx, vy, wz)         饱和标志      轮速 (左, 右)\n");
    for (int i = 0; i <= 60; ++i) {
        auto r  = limiter.limit(cmd, dt);
        auto ws = chassis.inverse_kinematics(r.out_);
        if (i % 10 == 0 || i == 60) {
            printf("%3d  (%6.3f, %6.3f, %6.3f)   %c%c%c        (%7.2f, %7.2f)\n",
                   i,
                   r.out_.vx_, r.out_.vy_, r.out_.wz_,
                   r.is_vx_lim_ ? 'V' : '-',
                   r.is_vy_lim_ ? 'V' : '-',
                   r.is_wz_lim_ ? 'V' : '-',
                   ws.values_[0], ws.values_[1]);
        }
    }
    printf("\n终态锚点: ωL = (0.5 − 0.08)/0.03 = 14.00,  ωR = (0.5 + 0.08)/0.03 = 19.33\n");
    return 0;
}