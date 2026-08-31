// L3 仿真验证：三底盘 2D 位姿积分（走一圈回原点，验证内核互逆一致性）
// 链路: Twist 指令 → inverse → 轮速 → forward(测量) → 欧拉积分 → (x,y,θ)
// 判据: 终点位置/角度误差 < 5e-3（理想轮下应精确为 0，误差=积分近似）
#include "chassis.hpp"
#include <cstdio>
#include <cmath>

struct Leg { Twist cmd; float seconds; };

template<typename Chassis>
bool simulate(const char* name, Chassis& ch, const Leg* legs, int n, float dt) {
    float x = 0, y = 0, theta = 0;
    int   grid[20][30] = {};
    auto plot = [&](float px, float py) {
        int gx = (int)((px + 0.6f) * 25.0f);   // x∈[-0.6,0.6] → 30 列
        int gy = (int)((py + 0.4f) * 25.0f);   // y∈[-0.4,0.4] → 20 行
        if (gx >= 0 && gx < 30 && gy >= 0 && gy < 20) grid[gy][gx] = 1;
    };

    printf("== %s ==\n", name);
    for (int i = 0; i < n; ++i) {
        int steps = (int)(legs[i].seconds / dt);
        for (int s = 0; s < steps; ++s) {
            WheelSpeeds ws   = ch.inverse_kinematics(legs[i].cmd);
            Twist       meas = ch.forward_kinematics(ws);
            x     += meas.vx_ * std::cos(theta) * dt - meas.vy_ * std::sin(theta) * dt;
            y     += meas.vx_ * std::sin(theta) * dt + meas.vy_ * std::cos(theta) * dt;
            theta += meas.wz_ * dt;
            plot(x, y);
        }
        printf("  第 %d 段: (%.4f, %.4f, θ=%.4f)\n", i + 1, x, y, theta);
    }

    float err = std::sqrt(x * x + y * y);
    float aerr = std::fabs(theta - 2.0f * 3.14159265f);  // 目标: 累计转 2π
    bool pass = err < 5e-3f && aerr < 5e-3f;
    printf("  终点 (%.4f, %.4f, θ=%.4f)  位置误差 %.5f m  角度误差 %.5f rad  → %s\n",
           x, y, theta, err, aerr, pass ? "PASS" : "FAIL");
    printf("  轨迹: ");
    for (int gy = 19; gy >= 0; --gy) {
        for (int gx = 0; gx < 30; ++gx) putchar(grid[gy][gx] ? '*' : '.');
        if (gy > 0) printf("/");
    }
    printf("\n\n");
    return pass;
}

int main() {
    const float dt = 0.01f;

    // ---- DiffDrive: 正方形（前进 0.3m + 原地转 90°）× 4 ----
    DiffDrive diff(0.16f, 0.03f);
    const Leg diff_legs[] = {
        {{0.3f, 0.0f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
        {{0.3f, 0.0f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
        {{0.3f, 0.0f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
        {{0.3f, 0.0f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
    };

    // ---- MecanumDrive: 回字形（前进→横移→后退→横移回）+ 转 360° ----
    MecanumDrive mec(0.1f, 0.1f, 0.03f);
    const Leg mec_legs[] = {
        {{0.3f, 0.0f, 0.0f}, 1.0f}, {{0.0f, 0.3f, 0.0f}, 1.0f},
        {{-0.3f, 0.0f, 0.0f}, 1.0f}, {{0.0f, -0.3f, 0.0f}, 1.0f},
        {{0.0f, 0.0f, 3.1415926f}, 2.0f},
    };

    // ---- OmniDrive: 菱形斜线（(0.3,0.3) × 4 边）+ 转 360° ----
    OmniDrive omni(3, 0.15f, 0.0f, 0.03f);
    const Leg omni_legs[] = {
        {{0.3f, 0.3f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
        {{0.3f, 0.3f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
        {{0.3f, 0.3f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
        {{0.3f, 0.3f, 0.0f}, 1.0f}, {{0.0f, 0.0f, 1.5707963f}, 1.0f},
    };

    bool ok = true;
    ok &= simulate("DiffDrive 正方形", diff, diff_legs, 8, dt);
    ok &= simulate("MecanumDrive 回字形+旋转", mec, mec_legs, 5, dt);
    ok &= simulate("OmniDrive 菱形", omni, omni_legs, 8, dt);
    printf("L3 仿真总判据: %s\n", ok ? "ALL PASS ✅" : "FAIL ❌");
    return ok ? 0 : 1;
}
