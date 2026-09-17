// test_wheel.cpp —— 五组件锚点测试 + Wheel 集成测试
// 锚点原则：测行为不测数值实现（数值锚点全部来自手算/legacy 参考）
#include "lpf.hpp"
#include "ramp.hpp"
#include "smooth_planner.hpp"
#include "pid.hpp"
#include "wheel.hpp"

#include <cstdio>
#include <cmath>

// 命名空间（2026-09-17）：全仓类型收进 lunokhod::（规则见 AGENTS.md §3）
using namespace lunokhod::wheel;
using namespace lunokhod;



static int g_fails = 0;
static bool close(float a, float b, float tol = 1e-3f) { return std::fabs(a - b) <= tol; }
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); ++g_fails; } } while(0)

// ============ 1. LPF：alpha = dt/(Tf+dt) = 1/3（Tf=0.02, dt=0.01）============
static void test_lpf() {
    LPF lpf(0.02f);
    CHECK(close(lpf.calc(1.0f, 0.01f), 0.333333f));   // 帧1: 1/3
    CHECK(close(lpf.calc(1.0f, 0.01f), 0.555556f));   // 帧2: 1/3 + 2/3·(1/3)
    CHECK(close(lpf.calc(1.0f, 0.01f), 0.703704f));   // 帧3: 1/3 + 2/3·(5/9)
    lpf.reset();
    CHECK(close(lpf.calc(1.0f, 0.01f), 0.333333f));   // reset 后重新从 0 滤波
}

// ============ 2. Ramp：max_rate=1500, dt=0.02 → step=30 ============
static void test_ramp() {
    Ramp ramp(1500.0f);
    float out = 0.0f;
    for (int i = 0; i < 33; ++i) out = ramp.calc(1000.0f, 0.02f);
    CHECK(close(out, 990.0f));                        // 帧33: 33×30 = 990
    out = ramp.calc(1000.0f, 0.02f);
    CHECK(close(out, 1000.0f));                       // 帧34: 爬满（0.68s，锚点）
    CHECK(close(ramp.calc(1000.0f, 0.02f), 1000.0f)); // 端点直通：目标不变输出不变
    ramp.reset();
    CHECK(close(ramp.calc(500.0f, 0.02f), 30.0f));    // reset 后重新从 0 爬（每帧最多 30）
}

// ============ 3. SmoothPlanner：Ramp + 两级 LPF，200 帧收敛 1000 无超调 ============
static void test_smooth_planner() {
    SmoothPlanner sp(1500.0f, 0.02f);
    float out = 0.0f, max_out = 0.0f;
    for (int i = 0; i < 200; ++i) {
        out = sp.calc(1000.0f, 0.01f);
        if (out > max_out) max_out = out;
    }
    CHECK(close(out, 1000.0f, 1e-1f));   // 200 帧（2s）内收敛
    CHECK(max_out <= 1000.0f + 1e-1f);   // 无超调（S 曲线平滑性）
    sp.reset();
    // reset 后重新爬：ramp 15/帧 → LPF1 (1/3)·15=5 → LPF2 (1/3)·5=1.67（两级滤波衰减）
    CHECK(close(sp.calc(1000.0f, 0.01f), 1.6667f, 1e-2f));
}

// ============ 4. PID：8 项行为锚点 ============
static void test_pid() {
    // 4.1 纯 P：kp=1 → out = error
    const PIDConfig c = PIDConfig{}.kp(1.0f).limit_out(1e6f);
    PID p(c);
    CHECK(close(p.calc(100.0f, 0.0f, 0.01f), 100.0f));

    // 4.2 对称钳位：limit_out=50 → 100 钳到 50
    const PIDConfig c2 = PIDConfig{}.kp(1.0f).limit_out(50.0f);
    PID p2(c2);
    CHECK(close(p2.calc(100.0f, 0.0f, 0.01f), 50.0f));

    // 4.3 梯形积分：error 恒定 10, ki=1, dt=0.01
    //     帧1: 0 + 1·0.01·0.5·(10+0)  = 0.05
    //     帧2: 0.05 + 1·0.01·0.5·(10+10) = 0.15
    //     帧3: 0.15 + 0.1 = 0.25
    const PIDConfig c3 = PIDConfig{}.ki(1.0f).limit_out(1e6f).limit_i(1e6f);
    PID p3(c3);
    CHECK(close(p3.calc(10.0f, 0.0f, 0.01f), 0.05f));
    CHECK(close(p3.calc(10.0f, 0.0f, 0.01f), 0.15f));
    CHECK(close(p3.calc(10.0f, 0.0f, 0.01f), 0.25f));

    // 4.4 积分分离：thresh=5, |error|=10 > 5 → 积分冻结
    const PIDConfig c4 = PIDConfig{}.ki(1.0f).thresh_i_sep(5.0f).limit_out(1e6f).limit_i(1e6f);
    PID p4(c4);
    CHECK(close(p4.calc(10.0f, 0.0f, 0.01f), 0.0f));
    CHECK(close(p4.calc(10.0f, 0.0f, 0.01f), 0.0f));   // 一直冻结

    // 4.5 微分先行：目标突变但 measure 未动 → D=0（无微分冲击）
    const PIDConfig c5 = PIDConfig{}.kd(1.0f).limit_out(1e6f);
    PID p5(c5);
    CHECK(close(p5.calc(0.0f, 0.0f, 0.01f), 0.0f));   // 首拍 error=0 → 输出 0（建立 measure_prev_）
    CHECK(close(p5.calc(100.0f, 0.0f, 0.01f), 0.0f));

    // 4.6 输出斜坡开关：max_rate=0 → 直通
    const PIDConfig c6 = PIDConfig{}.kp(1.0f).max_rate_out(0.0f).limit_out(1e6f);
    PID p6(c6);
    CHECK(close(p6.calc(100.0f, 0.0f, 0.01f), 100.0f));

    // 4.7 输出斜坡：1000/s × 0.01 = 10/帧
    const PIDConfig c7 = PIDConfig{}.kp(1.0f).max_rate_out(1000.0f).limit_out(1e6f);
    PID p7(c7);
    CHECK(close(p7.calc(100.0f, 0.0f, 0.01f), 10.0f));
    CHECK(close(p7.calc(100.0f, 0.0f, 0.01f), 20.0f));

    // 4.8 reset：积分清零，重新从 0.05 爬
    PID p8(c3);
    CHECK(close(p8.calc(10.0f, 0.0f, 0.01f), 0.05f));   // 预热两拍：同 4.3 算例 0.05 → 0.15
    CHECK(close(p8.calc(10.0f, 0.0f, 0.01f), 0.15f));
    p8.reset();
    CHECK(close(p8.calc(10.0f, 0.0f, 0.01f), 0.05f));
}

// ============ 5. Wheel：回调 + 组合链路 ============
static float g_measured = 0.0f;
static int16_t g_pwm_out = 0;
static int g_pwm_calls = 0;
static float fake_measure(uint8_t id, float dt) { (void)id; (void)dt; return g_measured; }
static void fake_set_pwm(uint8_t id, int16_t pwm) { (void)id; g_pwm_out = pwm; ++g_pwm_calls; }

static void test_wheel() {
    // 轮速环出厂配置（legacy 实测值）
    const PIDConfig cfg = PIDConfig{}
            .kp(3.0f).ki(1.0f).kd(0.0f)
            .limit_out(3600.0f).limit_i(1200.0f)
            .thresh_i_sep(20.0f).max_rate_out(10000.0f).d_filter_Tf(0.005f);

    SmoothPlannerConfig pc;             // 默认 1500/0.02
    Wheel w(0, pc, cfg, fake_measure, fake_set_pwm);

    // 5.1 初始：未输出、速度为 0
    CHECK(g_pwm_calls == 0);
    CHECK(w.get_speed() == 0.0f);

    // 5.2 set_cmd(1000) → 100 帧：PWM 爬坡输出（S 曲线 + PID 生效）
    w.set_cmd(1000.0f);
    float prev = -1.0f; bool ramped = false;
    for (int i = 0; i < 100; ++i) {
        w.update(0.01f);
        if (g_pwm_calls > 1 && g_pwm_out > prev) ramped = true;
        prev = g_pwm_out;
    }
    CHECK(g_pwm_calls == 100);
    CHECK(g_pwm_out > 0);
    CHECK(ramped);
    CHECK(w.effort() == g_pwm_out);          // 最近一次算出的 effort = 回调收到的值（外部观察者）

    // 5.3 测速透传：回调返回值 → get_speed()
    g_measured = 500.0f;
    w.update(0.01f);
    CHECK(w.get_speed() == 500.0f);

    // 5.4 stop：立即置 0；测速归零后再 update 输出保持 0
    w.stop();
    CHECK(g_pwm_out == 0);
    CHECK(w.effort() == 0);                  // stop() 之后归零
    CHECK(g_pwm_calls == 102);               // 101（上面那次 5.3 的 update）+ 1（stop 写 0）
    g_measured = 0.0f;
    w.update(0.01f);
    CHECK(g_pwm_out == 0);

    // 5.5 空回调保护：nullptr 不崩
    Wheel w2(1, pc, cfg, nullptr, nullptr);
    w2.set_cmd(100.0f);
    w2.update(0.01f);
    w2.stop();
    CHECK(w2.effort() == 0);                 // 空回调：不崩，且 stop 后仍是 0
}

int main() {
    test_lpf();
    test_ramp();
    test_smooth_planner();
    test_pid();
    test_wheel();

    if (g_fails == 0) { printf("ALL PASS\n"); return 0; }
    printf("%d FAILED\n", g_fails);
    return g_fails;
}
