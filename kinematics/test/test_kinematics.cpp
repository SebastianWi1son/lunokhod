#include "chassis.hpp"
#include <cstdio>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)
static bool close(float a, float b) { return (a - b < 1e-5f) && (b - a < 1e-5f); }

static void test_diff() {
    DiffDrive chassis(0.16f, 0.03f);          // 轮距 0.16m，轮半径 0.03m
    // 锚点：直行 0.5/0.03 = 50/3
    auto s1 = chassis.inverse_kinematics({0.5f, 0.0f, 0.0f});
    CHECK(s1.count_ == 2);
    CHECK(close(s1.values_[0], 50.0f/3.0f));
    CHECK(close(s1.values_[1], 50.0f/3.0f));
    // 锚点：原地转 ±0.16/(2×0.03) = ±8/3
    auto s2 = chassis.inverse_kinematics({0.0f, 0.0f, 1.0f});
    CHECK(close(s2.values_[0], -8.0f/3.0f));
    CHECK(close(s2.values_[1], +8.0f/3.0f));
    // 往返：forward(inverse(x)) ≈ x
    Twist cmd{0.5f, 0.0f, 1.0f};
    auto back = chassis.forward_kinematics(chassis.inverse_kinematics(cmd));
    CHECK(close(back.vx_, 0.5f));
    CHECK(close(back.wz_, 1.0f));
    CHECK(close(back.vy_, 0.0f));
    // 边界：零输入 → 全零
    auto s0 = chassis.inverse_kinematics({0.0f, 0.0f, 0.0f});
    CHECK(close(s0.values_[0], 0.0f) && close(s0.values_[1], 0.0f));
}

static void test_mecanum() {
    MecanumDrive mec(0.1f, 0.1f, 0.03f);      // lx=ly=0.1m, 轮半径 0.03m
    // 锚点：直行 → 四轮相等 = 50/3
    auto s1 = mec.inverse_kinematics({0.5f, 0.0f, 0.0f});
    CHECK(s1.count_ == 4);
    for (int i = 0; i < 4; ++i)
        CHECK(close(s1.values_[i], 50.0f/3.0f));
    // 锚点：横移 0.5 → {-,+,+,-}
    auto s2 = mec.inverse_kinematics({0.0f, 0.5f, 0.0f});
    CHECK(close(s2.values_[0], -50.0f/3.0f));
    CHECK(close(s2.values_[1], +50.0f/3.0f));
    CHECK(close(s2.values_[2], +50.0f/3.0f));
    CHECK(close(s2.values_[3], -50.0f/3.0f));
    // 锚点：旋转 1 → ±(lx+ly)/r = ±0.2/0.03 = ±20/3（-,+,-,+）
    auto s3 = mec.inverse_kinematics({0.0f, 0.0f, 1.0f});
    CHECK(close(s3.values_[0], -20.0f/3.0f));
    CHECK(close(s3.values_[1], +20.0f/3.0f));
    CHECK(close(s3.values_[2], -20.0f/3.0f));
    CHECK(close(s3.values_[3], +20.0f/3.0f));
    // 往返：三个通道全验
    Twist cmd{0.3f, 0.2f, 0.5f};
    auto back = mec.forward_kinematics(mec.inverse_kinematics(cmd));
    CHECK(close(back.vx_, 0.3f));
    CHECK(close(back.vy_, 0.2f));
    CHECK(close(back.wz_, 0.5f));
    // 边界
    auto s0 = mec.inverse_kinematics({0.0f, 0.0f, 0.0f});
    for (int i = 0; i < 4; ++i)
        CHECK(close(s0.values_[i], 0.0f));
}

static void test_omni() {
    OmniDrive omni(3, 0.15f, 0.0f, 0.03f);    // n=3, R=0.15m, γ=0, 轮半径 0.03m
    // 锚点：直行 0.5 → {0, -√3·25/3, +√3·25/3}
    auto s1 = omni.inverse_kinematics({0.5f, 0.0f, 0.0f});
    CHECK(s1.count_ == 3);
    CHECK(close(s1.values_[0], 0.0f));
    CHECK(close(s1.values_[1], -14.433757f));     // -0.8660/0.03·0.5
    CHECK(close(s1.values_[2], +14.433757f));
    // 锚点：横移 0.5 → {50/3, -25/3, -25/3}
    auto s2 = omni.inverse_kinematics({0.0f, 0.5f, 0.0f});
    CHECK(close(s2.values_[0], +50.0f/3.0f));
    CHECK(close(s2.values_[1], -25.0f/3.0f));
    CHECK(close(s2.values_[2], -25.0f/3.0f));
    // 锚点：旋转 1 → 三轮相等 = R/r = 5.0
    auto s3 = omni.inverse_kinematics({0.0f, 0.0f, 1.0f});
    for (int i = 0; i < 3; ++i)
        CHECK(close(s3.values_[i], 5.0f));
    // 往返（同时检验 forward 特例的符号约定）
    Twist cmd{0.3f, 0.2f, 0.5f};
    auto back = omni.forward_kinematics(omni.inverse_kinematics(cmd));
    CHECK(close(back.vx_, 0.3f));
    CHECK(close(back.vy_, 0.2f));
    CHECK(close(back.wz_, 0.5f));
    // 边界
    auto s0 = omni.inverse_kinematics({0.0f, 0.0f, 0.0f});
    for (int i = 0; i < 3; ++i)
        CHECK(close(s0.values_[i], 0.0f));

    // 通用 forward 验证（修复后）：n=4 互逆 + γ≠0 互逆
    OmniDrive omni4(4, 0.15f, 0.0f, 0.03f);
    auto back4 = omni4.forward_kinematics(omni4.inverse_kinematics(cmd));
    CHECK(close(back4.vx_, 0.3f));
    CHECK(close(back4.vy_, 0.2f));
    CHECK(close(back4.wz_, 0.5f));
    OmniDrive omni_g(3, 0.15f, 3.14159265f / 6.0f, 0.03f);   // γ=30°
    auto backg = omni_g.forward_kinematics(omni_g.inverse_kinematics(cmd));
    CHECK(close(backg.vx_, 0.3f));
    CHECK(close(backg.vy_, 0.2f));
    CHECK(close(backg.wz_, 0.5f));
}

int main() {
    test_diff();
    test_mecanum();
    test_omni();
    if (fails == 0) printf("ALL PASS\n");
    else printf("%d FAILED\n", fails);
    return fails;
}