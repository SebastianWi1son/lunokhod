// test_chassis_loop.cpp —— 装配层锚点测试
//
// ── oracle 说明（每个期望值从哪来）──────────────────────────────────────────
//   手算锚点  ：给定几何 + 阶跃命令 → 稳态轮目标由有理数手算得出（0.3/0.03 = 10、
//               0.22/0.03 = 22/3）；限幅到位的拍数 = ceil(1.0 / (1.5×0.01)) = 67
//   性质      ：互逆（twist() == cmd()）、每拍增量 ≤ acc·dt、不超调
//   穿透      ：dt / now 必须是【调用方给的那一份】—— 抓"装配层自己发明时间基"
//   记录      ：用会记录顺序的假 IO 抓 tick 的执行顺序与"未用到的轮子不被触碰"
//
// 三条硬约束都守着：期望值不由被测对象算、输入不由被测对象生成、容差有推导。
#include "chassis.hpp"
#include "chassis_loop.hpp"

#include <cmath>
#include <cstdio>

// 容差推导：float 相对 eps = 1.19e-7；本例算术最多 3 次乘除 → 累积 ≈ 3.6e-7
//          量级最大 10.0 → 绝对 3.6e-6 → 取 4 倍安全系数 → 1.5e-5，圆整为 2e-5
static const float TOL = 2e-5f;

static int g_fails = 0;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); ++g_fails; } } while (0)
static bool close(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// ── 假 IO：既供数，也记录"谁在什么时候被调、参数是多少" ─────────────────────
enum EvKind { EV_MEASURE, EV_PWM };
struct Ev { EvKind kind; uint8_t id; float dt; int16_t pwm; };

static const int MAX_EV = 512;
static Ev    g_ev[MAX_EV];
static int    g_ev_n = 0;
static float  g_meas[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // 由测试【喂】进去的实测轮速
static float  g_last_dt = -1.0f;

static void ev_reset() { g_ev_n = 0; g_last_dt = -1.0f; }
static void zero_meas() { for (int i = 0; i < 4; ++i) { g_meas[i] = 0.0f; } }

static float fake_measure(uint8_t id, float dt) {
    if (g_ev_n < MAX_EV) { g_ev[g_ev_n++] = Ev{EV_MEASURE, id, dt, 0}; }
    g_last_dt = dt;
    return g_meas[id];
}
static void fake_set_pwm(uint8_t id, int16_t pwm) {
    if (g_ev_n < MAX_EV) { g_ev[g_ev_n++] = Ev{EV_PWM, id, 0.0f, pwm}; }
}

// ── 里程计记录出口的捕获 ───────────────────────────────────────────────────
static int         g_sink_n = 0;
static uint32_t    g_sink_tick = 0;
static float       g_sink_dt = 0.0f;
static WheelSpeeds g_sink_ws{};
static Twist       g_sink_cmd{0.0f, 0.0f, 0.0f};
static Twist       g_sink_twist{0.0f, 0.0f, 0.0f};
static Pose        g_sink_pose{0.0f, 0.0f, 0.0f};

static void capture_sink(void* ctx, const OdometrySample& s) {
    (void)ctx;
    ++g_sink_n;
    g_sink_tick  = s.tick_;
    g_sink_dt    = s.dt_;
    g_sink_ws    = s.ws_;
    g_sink_cmd   = s.cmd_;
    g_sink_twist = s.twist_;
    g_sink_pose  = s.pose_;
}

// ── 公共配置：默认不限幅（本组测试里限幅另有专测）──────────────────────────
static ChassisLoopConfig unlimited_cfg() {
    ChassisLoopConfig c;
    c.acc_vx_ = 1.0e6f;
    c.acc_vy_ = 1.0e6f;
    c.acc_wz_ = 1.0e6f;
    c.pid_        = PIDConfig{};          // 全 0：PID 不参与（轮速由假 IO 直接给）
    c.planner_    = SmoothPlannerConfig{};
    c.twist_scale_ = 1.0f;
    return c;
}

// ============ 1. 逆解手算锚点（装配层必须把 cmd 原样喂给逆解）============
// 几何：r=0.03, lx=0.10, ly=0.12 → (lx+ly)/r = 0.22/0.03 = 22/3
static void test_inverse_hand_calc() {
    zero_meas();
    ChassisLoop<MecanumDrive> loop(unlimited_cfg(),
                                   MecanumDrive(0.10f, 0.12f, 0.03f),
                                   fake_measure, fake_set_pwm);

    // ① 纯前进 vx=0.3 → 四轮 = vx/r = 0.3/0.03 = 10.0
    loop.set_cmd(Twist{0.3f, 0.0f, 0.0f});
    loop.tick(0.01f, 1000u);
    for (uint8_t i = 0; i < 4; ++i) { CHECK(close(loop.wheel_target(i), 10.0f, TOL)); }

    // ② 纯自转 wz=1 → 四轮 = ∓(lx+ly)/r = ∓22/3
    loop.set_cmd(Twist{0.0f, 0.0f, 1.0f});
    loop.tick(0.01f, 1010u);
    CHECK(close(loop.wheel_target(0), -22.0f / 3.0f, TOL));
    CHECK(close(loop.wheel_target(1),  22.0f / 3.0f, TOL));
    CHECK(close(loop.wheel_target(2), -22.0f / 3.0f, TOL));
    CHECK(close(loop.wheel_target(3),  22.0f / 3.0f, TOL));

    // ③ 混合 vx + wz → 10 ∓ 22/3 = 8/3 与 52/3
    loop.set_cmd(Twist{0.3f, 0.0f, 1.0f});
    loop.tick(0.01f, 1020u);
    CHECK(close(loop.wheel_target(0),  8.0f / 3.0f, TOL));
    CHECK(close(loop.wheel_target(1), 52.0f / 3.0f, TOL));
    CHECK(close(loop.wheel_target(2),  8.0f / 3.0f, TOL));
    CHECK(close(loop.wheel_target(3), 52.0f / 3.0f, TOL));
}

// ============ 2. 限幅：每拍增量 ≤ acc·dt、不超调、到位的拍数（手算 67）====
static void test_acc_limit_property_and_tick_count() {
    zero_meas();
    ChassisLoopConfig cfg = unlimited_cfg();
    cfg.acc_vx_ = 1.5f;
    ChassisLoop<MecanumDrive> loop(cfg, MecanumDrive(0.10f, 0.12f, 0.03f),
                                   fake_measure, fake_set_pwm);

    const float dt = 0.01f;             // max_step = 1.5 × 0.01 = 0.015 m/s 每拍
    loop.set_cmd(Twist{1.0f, 0.0f, 0.0f});

    int   k = 0;
    float prev = 0.0f;
    while (loop.cmd().vx_ < 1.0f && k < 1000) {
        loop.tick(dt, 1000u + 10u * static_cast<uint32_t>(k));
        const float v = loop.cmd().vx_;
        CHECK(v - prev <= cfg.acc_vx_ * dt + 1e-6f);   // 性质：不能一步跳到位
        CHECK(v <= 1.0f);                              // 性质：不超调
        prev = v;
        ++k;
    }
    // 手算：ceil(1.0 / 0.015) = 67 拍（限幅器状态跨拍保持；每拍清零的话永远到不了）
    CHECK(k == 67);
    CHECK(loop.cmd().vx_ == 1.0f);
}

// ============ 3. cmd() = 限幅后；且限幅必须在逆解【之前】==================
static void test_cmd_is_post_limit_and_limit_precedes_inverse() {
    zero_meas();
    ChassisLoopConfig cfg = unlimited_cfg();
    cfg.acc_vx_ = 1.5f;
    ChassisLoop<MecanumDrive> loop(cfg, MecanumDrive(0.10f, 0.12f, 0.03f),
                                   fake_measure, fake_set_pwm);

    loop.set_cmd(Twist{1.0f, 0.0f, 0.0f});      // 上游要 1.0 m/s
    loop.tick(0.01f, 1000u);

    // cmd() 必须是【限幅后】的 1.5 × 0.01 = 0.015（若写成限幅前的 1.0 → 这里红）
    CHECK(close(loop.cmd().vx_, 0.015f, 1e-6f));
    CHECK(loop.cmd().vx_ != 1.0f);

    // 逆解吃到的必须是限幅后的值：0.015/0.03 = 0.5 rad/s（限幅在逆解后 → 这里红）
    for (uint8_t i = 0; i < 4; ++i) { CHECK(close(loop.wheel_target(i), 0.5f, 1e-6f)); }

    // 轮子没转 → 正解 = 0 → twist() ≠ cmd()（L0 残差有意义，不是恒等）
    CHECK(loop.twist().vx_ == 0.0f);
}

// ============ 4. 执行顺序 + 未用到的轮子不被触碰（决策 3）================
static void test_order_and_unused_wheels() {
    zero_meas();
    ChassisLoop<DiffDrive> loop(unlimited_cfg(), DiffDrive(0.24f, 0.03f),
                                fake_measure, fake_set_pwm);
    ev_reset();
    loop.set_cmd(Twist{0.2f, 0.0f, 0.1f});
    for (int i = 0; i < 5; ++i) { loop.tick(0.01f, 1000u + 10u * static_cast<uint32_t>(i)); }

    int meas_cnt[4] = {0, 0, 0, 0};
    int pwm_cnt[4]  = {0, 0, 0, 0};
    for (int e = 0; e < g_ev_n; ++e) {
        if (g_ev[e].kind == EV_MEASURE) { ++meas_cnt[g_ev[e].id]; }
        else                            { ++pwm_cnt[g_ev[e].id]; }
    }
    // 差速 = 2 轮：只有 0/1 被碰过，每拍各一次
    CHECK(meas_cnt[0] == 5); CHECK(meas_cnt[1] == 5);
    CHECK(pwm_cnt[0]  == 5); CHECK(pwm_cnt[1]  == 5);
    CHECK(meas_cnt[2] == 0); CHECK(meas_cnt[3] == 0);
    CHECK(pwm_cnt[2]  == 0); CHECK(pwm_cnt[3]  == 0);

    // 每拍每轮：先测速（④ 在 ③ 的 update() 内部）后写 PWM，且同一轮
    CHECK(g_ev_n == 20);
    for (int e = 0; e + 1 < g_ev_n; e += 2) {
        CHECK(g_ev[e].kind == EV_MEASURE);
        CHECK(g_ev[e + 1].kind == EV_PWM);
        CHECK(g_ev[e].id == g_ev[e + 1].id);
    }

    // 没被用到的轮：只读口也必须是干净的 0（不许漏 indeterminate 出去）
    CHECK(loop.wheel_target(2) == 0.0f);
    CHECK(loop.wheel_target(3) == 0.0f);
    CHECK(loop.wheel_speed(2) == 0.0f);
    CHECK(loop.wheel_speed(3) == 0.0f);
}

// ============ 5. dt / now 穿透：装配层不发明时间基 ========================
static void test_dt_and_now_passthrough() {
    zero_meas();
    ChassisLoop<MecanumDrive> loop(unlimited_cfg(), MecanumDrive(0.10f, 0.12f, 0.03f),
                                   fake_measure, fake_set_pwm);
    loop.set_sink(capture_sink, nullptr);
    loop.set_cmd(Twist{0.1f, 0.0f, 0.0f});
    ev_reset();
    g_sink_n = 0;

    loop.tick(0.013f, 1234567u);          // dt 故意偏离标称的 0.001
    CHECK(g_last_dt == 0.013f);           // 实测 dt 原样喂给编码器读取
    CHECK(g_sink_n == 1);
    CHECK(g_sink_dt == 0.013f);           // 也原样进里程计
    CHECK(g_sink_tick == 1234567u);       // now 原样进里程计（换成内部计数器 → 这里红）

    loop.tick(0.007f, 999999u);           // 故意回拨 —— 装配层只传不判，不许崩
    CHECK(g_sink_dt == 0.007f);
    CHECK(g_sink_tick == 999999u);

    // 只读口自洽：记录里的每一笔都必须与只读口一致
    CHECK(g_sink_ws.count_ == 4);
    for (uint8_t i = 0; i < 4; ++i) { CHECK(g_sink_ws.values_[i] == loop.wheel_speed(i)); }
    CHECK(g_sink_cmd.vx_ == loop.cmd().vx_);
    CHECK(g_sink_twist.vx_ == loop.twist().vx_);          // twist_scale_ = 1.0 → 恒等
    CHECK(g_sink_pose.x_ == loop.pose().x_);
    CHECK(g_sink_pose.yaw_ == loop.pose().yaw_);
}

// ============ 7. 逆解结果必须真的下发到轮子（专抓"忘了 set_cmd"）========
// 让"目标 → PWM"完全可手算：
//   Ramp 不饱和（max_rate 极大）+ LPF 的 Tf=0 → alpha = dt/dt = 1 → planner 直通
//   kp = 1 → PWM = 目标转速（截断到 int16）
static int16_t last_pwm(uint8_t id) {
    for (int e = g_ev_n - 1; e >= 0; --e) {
        if (g_ev[e].kind == EV_PWM && g_ev[e].id == id) { return g_ev[e].pwm; }
    }
    return 0;
}

static void test_targets_actually_reach_wheels() {
    zero_meas();
    ChassisLoopConfig cfg = unlimited_cfg();
    cfg.planner_.max_rate_ = 1.0e6f;      // Ramp 直通
    cfg.planner_.Tf_       = 0.0f;        // LPF 直通（alpha = dt/(Tf+dt) = 1）
    cfg.pid_.kp_           = 1.0f;        // PWM = 目标
    cfg.pid_.limit_out_    = 1.0e6f;
    cfg.pid_.max_rate_out_ = 0.0f;        // 0 = 输出斜坡关闭
    ChassisLoop<MecanumDrive> loop(cfg, MecanumDrive(0.10f, 0.12f, 0.03f),
                                   fake_measure, fake_set_pwm);
    ev_reset();
    loop.set_cmd(Twist{0.3f, 0.3f, 0.3f});
    loop.tick(0.01f, 1000u);              // 第一拍：measure = 0 → error = planned = 目标

    // 手算（vx/r = vy/r = 10；(lx+ly)/r × wz = 0.22/0.03 × 0.3 = 2.2）：
    //   {-1,+1,-(lx+ly)}/r → 10 - 10 - 2.2 = -2.2 = -11/5
    //   {+1,+1,+(lx+ly)}/r → 10 + 10 + 2.2 = 22.2 = 111/5
    //   {+1,+1,-(lx+ly)}/r → 10 + 10 - 2.2 = 17.8 =  89/5
    //   {+1,-1,+(lx+ly)}/r → 10 - 10 + 2.2 =  2.2 =  11/5
    // 容差 ±1：PWM 截断到 int16（向零取整）
    CHECK(close(static_cast<float>(last_pwm(0)), -11.0f / 5.0f, 1.0f));
    CHECK(close(static_cast<float>(last_pwm(1)), 111.0f / 5.0f, 1.0f));
    CHECK(close(static_cast<float>(last_pwm(2)),  89.0f / 5.0f, 1.0f));
    CHECK(close(static_cast<float>(last_pwm(3)),  11.0f / 5.0f, 1.0f));
}

// ============ 6. 互逆：实测 = 目标 → twist() 回到 cmd() ==================
static void test_round_trip() {
    zero_meas();
    ChassisLoop<MecanumDrive> loop(unlimited_cfg(), MecanumDrive(0.10f, 0.12f, 0.03f),
                                   fake_measure, fake_set_pwm);
    const Twist want{0.25f, -0.15f, 0.4f};

    loop.set_cmd(want);
    loop.tick(0.01f, 1000u);              // 拍 1：定出轮目标
    for (uint8_t i = 0; i < 4; ++i) { g_meas[i] = loop.wheel_target(i); }

    loop.tick(0.01f, 1010u);              // 拍 2：编码器报"完美跟踪"
    CHECK(close(loop.twist().vx_, want.vx_, TOL));   // 性质：互逆（装配层不额外改数）
    CHECK(close(loop.twist().vy_, want.vy_, TOL));
    CHECK(close(loop.twist().wz_, want.wz_, TOL));

    for (uint8_t i = 0; i < 4; ++i) { CHECK(loop.wheel_speed(i) == g_meas[i]); }
    CHECK(close(loop.cmd().vx_, want.vx_, 1e-6f));   // cmd() 只反映命令，不受测量影响
    zero_meas();
}

int main() {
    test_inverse_hand_calc();
    test_acc_limit_property_and_tick_count();
    test_cmd_is_post_limit_and_limit_precedes_inverse();
    test_order_and_unused_wheels();
    test_dt_and_now_passthrough();
    test_round_trip();
    test_targets_actually_reach_wheels();

    if (g_fails == 0) { printf("ALL PASS\n"); return 0; }
    printf("%d FAILED\n", g_fails);
    return g_fails;
}
