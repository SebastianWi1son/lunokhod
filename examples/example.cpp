#include "chassis.hpp"
#include <cstdio>

static void print_ws(const char* name, const WheelSpeeds& ws) {
    printf("%-10s count=%u  [", name, (unsigned)ws.count_);
    for (uint8_t i = 0; i < ws.count_; ++i)
        printf("%s%.2f", i ? ", " : "", ws.values_[i]);
    printf("] rad/s\n");
}
static void print_twist(const char* name, const Twist& t) {
    printf("%-10s vx=%.3f  vy=%.3f  wz=%.3f\n", name, t.vx_, t.vy_, t.wz_);
}

int main() {
    printf("== lunokhod kinematics 使用演示 ==\n\n");

    // ===== 1. 差速底盘（轮距 0.16m，轮半径 0.03m）=====
    DiffDrive diff(0.16f, 0.03f);
    Twist cmd1{0.5f, 0.0f, 1.0f};                    // 指令：前进 0.5m/s + 自转 1rad/s
    printf("[DiffDrive]  cmd: 前进 0.5m/s + 旋转 1rad/s\n");
    WheelSpeeds ws1 = diff.inverse_kinematics(cmd1); // inverse = 下发指令给轮速环
    print_ws("wheel", ws1);
    Twist back1 = diff.forward_kinematics(ws1);      // forward = 里程计观测（编码器→速度）
    print_twist("recovered", back1);

    // ===== 2. Mecanum（lx=ly=0.1m，轮半径 0.03m）：全向移动，可横移 =====
    MecanumDrive mec(0.1f, 0.1f, 0.03f);
    Twist cmd2{0.3f, 0.4f, 0.0f};                    // 指令：前进 + 横移（差速做不到的）
    printf("\n[MecanumDrive]  cmd: 前进 0.3m/s + 横移 0.4m/s\n");
    WheelSpeeds ws2 = mec.inverse_kinematics(cmd2);
    print_ws("wheel", ws2);
    Twist back2 = mec.forward_kinematics(ws2);
    print_twist("recovered", back2);

    // ===== 3. Omni（3 轮，R=0.15m，γ=0，轮半径 0.03m）=====
    OmniDrive omni(3, 0.15f, 0.0f, 0.03f);
    Twist cmd3{0.0f, 0.5f, 0.0f};                    // 指令：纯横移
    printf("\n[OmniDrive]  cmd: 横移 0.5m/s\n");
    WheelSpeeds ws3 = omni.inverse_kinematics(cmd3);
    print_ws("wheel", ws3);
    Twist back3 = omni.forward_kinematics(ws3);
    print_twist("recovered", back3);

    printf("\nrecovered ≈ cmd 说明正逆运动学自洽（互逆性）\n");
    return 0;
}