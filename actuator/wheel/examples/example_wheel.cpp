// example_wheel.cpp —— 电机模型闭环仿真（L3 级验证，"看得见"的部分）
//
// 链路：Wheel.set_cmd(1000) → update → measure_speed 回调读 MotorModel
//       → PID 输出 effort → set_effort 回调喂回 MotorModel（一阶惯性）
// 锚点：0.7s 内收敛 ±5%（950~1050），无超调
//
// 单位说明：本示例数值单位 = rpm（legacy 参考单位）；Wheel 接口本身单位无关。
#include "wheel.hpp"

#include <cstdio>

// 命名空间（2026-09-17）：全仓类型收进 lunokhod::（规则见 AGENTS.md §3）
using namespace lunokhod::wheel;
using namespace lunokhod;



// 电机模型：PWM → rpm 一阶惯性（模拟真实电机动态）
// 满 PWM=3600 → 目标 3000rpm；时间常数 T=100ms
struct MotorModel {
    float rpm = 0.0f;
    float T = 0.1f;
    void step(float effort, float dt) {
        float target = effort / 3600.0f * 3000.0f;
        rpm += (target - rpm) * dt / (T + dt);
    }
};

// 真机回调形态：全局函数 + motor_id（电机库惯例，无捕获）
static MotorModel g_motor;
static int16_t g_last_effort = 0;
static float measure_speed(uint8_t id, float dt) { (void)id; (void)dt; return g_motor.rpm; }
static void   set_effort(uint8_t id, int16_t effort)   {
    (void)id;
    g_last_effort = effort;             // 本示例的平台：effort 直连 PWM 占空（±3600）
    g_motor.step(static_cast<float>(effort), 0.01f);   // 电机模型固定 1kHz 时间步（回调不带 dt，周期由上层固定）
}

int main() {
    // 闭环演示参数（示例自调：目标 0.7s 内收敛 ±5%、无超调）
    // 注意：不是 legacy 出厂值——legacy 参数配本文档的电机模型会因积分分离冻结在纯 P 静差（714rpm），
    // 真实系统参数与电机增益匹配，此处为演示闭环收敛而调校。
    const PIDConfig cfg = PIDConfig{}
            .kp(4.0f).ki(10.0f).kd(0.5f)
            .limit_out(3600.0f).limit_i(1200.0f)
            .thresh_i_sep(0.0f)                  // 关闭积分分离（启动即积分，避免纯 P 静差）
            .max_rate_out(10000.0f).d_filter_Tf(0.005f);

    SmoothPlannerConfig pc;
    pc.max_rate_ = 5000.0f;      // 加速爬满（0.2s），电机惯性 T=0.1s 是主要滞后
    pc.Tf_       = 0.02f;
    Wheel w(0, pc, cfg, measure_speed, set_effort);

    const float DT = 0.01f;               // 1kHz 控制周期
    w.set_cmd(1000.0f);                   // 目标 1000 rpm

    printf("t(s)     rpm  effort\n");
    printf("------ ------- -------\n");
    int converge_frame = -1;
    float max_rpm = -1e9f;

    for (int i = 1; i <= 200; ++i) {      // 2 秒仿真
        w.update(DT);
        float rpm = g_motor.rpm;
        if (rpm > max_rpm) max_rpm = rpm;
        if (converge_frame < 0 && rpm >= 950.0f && rpm <= 1050.0f) converge_frame = i;
        if (i % 20 == 0 || i == 1)
            printf("%5.2f  %7.1f  %7d\n", static_cast<float>(i) * DT, rpm, g_last_effort);
    }

    printf("------ ------- -------\n");
    printf("收敛帧: %d（锚点 ≤70 帧 = 0.7s）  峰值: %.1f rpm（锚点 ≤1050）\n",
           converge_frame, max_rpm);

    int rc = 0;
    if (converge_frame < 0 || converge_frame > 70) { printf("FAIL: 未在 0.7s 内收敛 ±5%%\n"); rc = 1; }
    if (max_rpm > 1050.0f) { printf("FAIL: 超调 > 5%%\n"); rc = 1; }
    if (rc == 0) printf("SIM PASS: 0.7s 内收敛 ±5%%，无超调\n");
    else printf("SIM FAIL\n");
    return rc;
}
