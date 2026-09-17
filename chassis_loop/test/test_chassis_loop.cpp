// ⚠ 本文件已切到【执行器组接缝版】（2026-09-17）。
//   在 inc/wheel_set.hpp + 新版 inc/chassis_loop.hpp 落地之前，**本目标编译不过** ——
//   这是预期状态，不是你的错（`grep -n wheel_set.hpp inc/` 为空时就是还没敲）。
//   落地后：cmake --build build -j && ctest --test-dir build --output-on-failure  → 应 9/9
//   施工单：chassis_loop/docs/WORK_SEAM.md
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
#include "wheel_set.hpp"   // 执行器组（它自带 chassis_loop.hpp）
#include "chassis.hpp"       // 底盘类型

#include <cmath>
#include <cstdio>

// 命名空间（2026-09-17）：全仓类型收进 lunokhod::（规则见 AGENTS.md §3）
using namespace lunokhod;
using namespace lunokhod::chassis_loop;
using namespace lunokhod::odometry;
using namespace lunokhod::kinematics;
using namespace lunokhod::wheel;


// P29：wheel 的类型在 `namespace wheel` 里（wheel_set.hpp 内部已用限定名）
using wheel::PIDConfig;
using wheel::SmoothPlannerConfig;

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
static float  g_meas[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // 由测试【喂】进去的实测轮速（容量 = 契约容量 6）
static float  g_last_dt = -1.0f;

static void ev_reset() { g_ev_n = 0; g_last_dt = -1.0f; }
static void zero_meas() { for (int i = 0; i < 6; ++i) { g_meas[i] = 0.0f; } }

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

// ── 公共配置 ───────────────────────────────────────────────────────────────
// 编排层：默认不限幅（本组测试里限幅另有专测）
static ChassisLoopConfig unlimited_cfg() {
    ChassisLoopConfig c;
    c.acc_vx_ = 1.0e6f;
    c.acc_vy_ = 1.0e6f;
    c.acc_wz_ = 1.0e6f;
    c.twist_scale_ = 1.0f;
    return c;
}

// 执行器组：PID 全 0（轮速由假 IO 直接给）
static WheelSetConfig neutral_wset_cfg() {
    WheelSetConfig w;
    w.pid_     = PIDConfig{};
    w.planner_ = SmoothPlannerConfig{};
    return w;
}

// 一句工厂：把「底盘 + 假 IO」打包成执行器组（测试里到处要）
template <typename Chassis>
static WheelSet<Chassis> wset(const Chassis& c, const WheelSetConfig& wc = neutral_wset_cfg()) {
    return WheelSet<Chassis>(wc, c, fake_measure, fake_set_pwm);
}

// ============ 1. 逆解手算锚点（装配层必须把 cmd 原样喂给逆解）============
// 几何：r=0.03, lx=0.10, ly=0.12 → (lx+ly)/r = 0.22/0.03 = 22/3
static void test_inverse_hand_calc() {
    zero_meas();
    WheelLoop<MecanumDrive> loop(unlimited_cfg(), wset(MecanumDrive(0.10f, 0.12f, 0.03f)));

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
    WheelLoop<MecanumDrive> loop(cfg, wset(MecanumDrive(0.10f, 0.12f, 0.03f)));

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
    WheelLoop<MecanumDrive> loop(cfg, wset(MecanumDrive(0.10f, 0.12f, 0.03f)));

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
    WheelLoop<DiffDrive> loop(unlimited_cfg(), wset(DiffDrive(0.24f, 0.03f)));
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
    WheelLoop<MecanumDrive> loop(unlimited_cfg(), wset(MecanumDrive(0.10f, 0.12f, 0.03f)));
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
    WheelSetConfig wcfg;                              // 执行器组的参数
    wcfg.planner_.max_rate_ = 1.0e6f;                 // Ramp 直通
    wcfg.planner_.Tf_       = 0.0f;                   // LPF 直通（alpha = dt/(Tf+dt) = 1）
    wcfg.pid_ = PIDConfig{}.kp(1.0f)                  // PWM = 目标（上游链式 API）
                           .limit_out(1.0e6f)
                           .max_rate_out(0.0f);       // 0 = 输出斜坡关闭
    WheelLoop<MecanumDrive> loop(cfg, wset(MecanumDrive(0.10f, 0.12f, 0.03f), wcfg));
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

    // 单轮 effort 暴露（P26 加的第 5 个契约方法）：必须与回调**实际收到**的值一致
    // （外部观察者做金标；不是拿被测对象算自己）
    CHECK(loop.wheel_effort(0) == last_pwm(0));
    CHECK(loop.wheel_effort(1) == last_pwm(1));
    CHECK(loop.wheel_effort(2) == last_pwm(2));
    CHECK(loop.wheel_effort(3) == last_pwm(3));
    CHECK(loop.wheel_effort(4) == 0);            // 未用到的轮子：从不 apply → 0
    CHECK(loop.wheel_effort(5) == 0);
}

// ============ 6. 互逆：实测 = 目标 → twist() 回到 cmd() ==================
static void test_round_trip() {
    zero_meas();
    WheelLoop<MecanumDrive> loop(unlimited_cfg(), wset(MecanumDrive(0.10f, 0.12f, 0.03f)));
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

// ============ 8. 边界 N：三轮 / 六轮（容量 = 契约上限）================
// oracle：
//   手算 —— OmniDrive 的 J 第三列 = cr/wr（与轮号无关）→ 只给 wz 时【每一轮】目标都相等
//   记录 —— 假 IO 数「哪些轮被碰过」，证明活跃数 = count_，其余轮一个都不碰
// ⚠ 越界是 UB：Debug 下可能侥幸通过，**只有消毒档（ASan）能必然抓住** ——
//   所以验收必跑 ASan + UBSan（本组就是 P24 的回归用例）。
static void test_omni_boundary_wheel_counts() {
    const float expect = 10.0f / 3.0f;      // cr/wr = 0.10/0.03

    // (a) 三轮（wn = 3）
    {
        zero_meas();
        ev_reset();
        WheelLoop<OmniDrive<3>> loop(unlimited_cfg(), wset(OmniDrive<3>(0.10f, 0.0f, 0.03f)));
        loop.set_cmd(Twist{0.0f, 0.0f, 1.0f});
        loop.tick(0.01f, 1000u);
        for (uint8_t i = 0; i < 3; ++i) { CHECK(close(loop.wheel_target(i), expect, TOL)); }
        for (uint8_t i = 3; i < 6; ++i) { CHECK(loop.wheel_target(i) == 0.0f); }
        int touched[6] = {0, 0, 0, 0, 0, 0};
        for (int e = 0; e < g_ev_n; ++e) {
            if (g_ev[e].kind == EV_MEASURE) { ++touched[g_ev[e].id]; }
        }
        for (uint8_t i = 0; i < 3; ++i) { CHECK(touched[i] == 1); }
        for (uint8_t i = 3; i < 6; ++i) { CHECK(touched[i] == 0); }
    }

    // (b) 六轮（wn = 6 = 契约容量上限；装配层容量必须 ≥ 它）
    {
        zero_meas();
        ev_reset();
        WheelLoop<OmniDrive<6>> loop(unlimited_cfg(), wset(OmniDrive<6>(0.10f, 0.0f, 0.03f)));
        loop.set_cmd(Twist{0.0f, 0.0f, 1.0f});
        loop.tick(0.01f, 1000u);
        for (uint8_t i = 0; i < 6; ++i) { CHECK(close(loop.wheel_target(i), expect, TOL)); }
        for (uint8_t i = 0; i < 6; ++i) { CHECK(loop.wheel_speed(i) == 0.0f); }
        int touched[6] = {0, 0, 0, 0, 0, 0};
        for (int e = 0; e < g_ev_n; ++e) {
            if (g_ev[e].kind == EV_MEASURE) { ++touched[g_ev[e].id]; }
        }
        for (uint8_t i = 0; i < 6; ++i) { CHECK(touched[i] == 1); }
    }
}

// ============ 9. 接缝一致性：换一个执行器组，装配层照跑 ==================
// 这一组存在的意义：证明「执行器组」这个模板参数是**真的**（不是装饰参数 ——
// 反例见 foucault 的 Estimator<EKF>：那个模板参数从没被用过）。
// 这个假执行器组不接任何硬件，四个动作全部手算得出来：
//   inverse: 目标 = {vx × 2, wz × 3}
//   apply  : 什么都不做
//   measure: 恒定反馈 {7, -1}
//   forward: vx = 反馈[0] × 0.5（vy / wz 恒 0，好让位姿能手算）
struct StubSet {
    WheelSpeeds inverse(const Twist& t) const {
        WheelSpeeds sp{};
        sp.count_ = 2;
        sp.values_[0] = t.vx_ * 2.0f;
        sp.values_[1] = t.wz_ * 3.0f;
        return sp;
    }
    void apply(const WheelSpeeds&, float) {}
    WheelSpeeds measure(const WheelSpeeds&) const {   // 活跃数取自设定值（v1 契约）；Stub 固定 2
        WheelSpeeds fb{};
        fb.count_ = 2;
        fb.values_[0] = 7.0f;
        fb.values_[1] = -1.0f;
        return fb;
    }
    Twist forward(const WheelSpeeds& fb) const {
        Twist t{0.0f, 0.0f, 0.0f};
        t.vx_ = fb.values_[0] * 0.5f;
        return t;
    }
    int16_t effort(uint8_t) const { return kStubEffort; }   // 第 5 个契约方法
    static constexpr int16_t kStubEffort = 42;
};

static void test_actuator_seam() {
    zero_meas();
    ChassisLoop<StubSet> loop(unlimited_cfg(), StubSet{});
    loop.set_cmd(Twist{1.0f, 0.0f, 1.0f});
    loop.tick(0.1f, 1000u);

    CHECK(loop.wheel_target(0) == 2.0f);       // vx × 2
    CHECK(loop.wheel_target(1) == 3.0f);       // wz × 3
    CHECK(loop.wheel_speed(0) == 7.0f);        // 恒定反馈
    CHECK(loop.wheel_speed(1) == -1.0f);
    CHECK(loop.twist().vx_ == 3.5f);           // 7 × 0.5
    CHECK(loop.twist().vy_ == 0.0f);
    CHECK(loop.twist().wz_ == 0.0f);
    CHECK(close(loop.pose().x_, 0.35f, TOL));  // 3.5 × 0.1（yaw = 0 → 纯前进）
    CHECK(loop.pose().y_ == 0.0f);
    CHECK(loop.pose().yaw_ == 0.0f);
    CHECK(loop.cmd().vx_ == 1.0f);             // 不限幅 → 原值
    CHECK(loop.wheel_effort(0) == 42);          // 单轮 effort 透传（契约第 5 个方法）
}

// ============ 10. 限幅结果暴露（P25）：饱和标志与限幅后输出都拿得到 ============
// 契约（DESIGN.md §7）：`limit_result()` 返回**上一拍 tick()** 的限幅结果；
// 首次 tick() 之前必须是**确定零值**（`LimitResult` 有默认成员初始化器），不是脏值。
static void test_limit_result_exposed() {
    zero_meas();
    ChassisLoopConfig cfg;                        // 默认 acc_vx = 1.5、acc_vy = 1.5、acc_wz = 4.0
    cfg.twist_scale_ = 1.0f;
    WheelLoop<MecanumDrive> loop(cfg, wset(MecanumDrive(0.10f, 0.12f, 0.03f)));

    // 10.1 首次 tick() 之前：零值（不是垃圾）—— 这一条就是为 `lim_res_{}` 立的行为锚
    CHECK(loop.limit_result().out_.vx_ == 0.0f);
    CHECK(loop.limit_result().out_.vy_ == 0.0f);
    CHECK(loop.limit_result().out_.wz_ == 0.0f);
    CHECK(loop.limit_result().is_vx_lim_ == false);
    CHECK(loop.limit_result().is_vy_lim_ == false);
    CHECK(loop.limit_result().is_wz_lim_ == false);

    // 10.2 阶跃 → 第一拍必被限：增量 = acc_vx × dt = 1.5 × 0.01 = 0.015（手算）
    const float dt = 0.01f;
    loop.set_cmd(Twist{1.0f, 0.0f, 0.0f});
    loop.tick(dt, 1000u);
    CHECK(loop.limit_result().is_vx_lim_ == true);
    CHECK(close(loop.limit_result().out_.vx_, 0.015f, 1e-6f));
    CHECK(loop.limit_result().is_vy_lim_ == false);      // 该通道没在动 → 不饱和
    CHECK(loop.limit_result().is_wz_lim_ == false);
    // 一致性（两条独立路径给同一事实）：limit_result().out_ == cmd()
    CHECK(loop.limit_result().out_.vx_ == loop.cmd().vx_);
    CHECK(loop.limit_result().out_.vy_ == loop.cmd().vy_);
    CHECK(loop.limit_result().out_.wz_ == loop.cmd().wz_);

    // 10.3 爬满之后不再饱和：1.0 / 0.015 ≈ 67 拍 → 给 70 拍
    for (int i = 0; i < 70; ++i) { loop.tick(dt, 1000u); }
    CHECK(loop.limit_result().is_vx_lim_ == false);
    CHECK(close(loop.cmd().vx_, 1.0f, 1e-5f));
}

// ============ 11. 契约：未用到的槽 / 轮恒为零（不依赖某个 Chassis 的实现）============
// 为什么需要"脏尾巴底盘"这种陪练：`kinematics` 自己也会零初始化（F1），所以拿本仓三种
// 底盘做用例，**装配层那道堵槽删掉也照样全绿**（实测盲区）。但"未用到的槽堵 0"是
// **执行器组契约**（DESIGN §5.2 的 inverse 承诺）—— 不能依赖某个具体 Chassis 的实现细节。
// 这个陪练故意返回脏尾巴，把契约变成可观测的。
struct DirtyTailDrive {
    static WheelSpeeds inverse_kinematics(const Twist& t) {
        WheelSpeeds sp{};
        sp.count_ = 2;
        sp.values_[0] = t.vx_ / 0.03f;
        sp.values_[1] = t.vx_ / 0.03f;
        for (int i = 2; i < 6; ++i) { sp.values_[i] = 999.0f; }   // ← 故意脏
        return sp;
    }
    static Twist forward_kinematics(const WheelSpeeds& fb) {
        Twist t{0.0f, 0.0f, 0.0f};
        t.vx_ = 0.03f * (fb.values_[0] + fb.values_[1]) / 2.0f;
        return t;
    }
};

static void test_unused_slots_are_zeroed_contract() {
    zero_meas();
    ev_reset();
    WheelLoop<DirtyTailDrive> loop(unlimited_cfg(), wset(DirtyTailDrive{}));
    loop.set_cmd(Twist{0.3f, 0.0f, 0.0f});
    loop.tick(0.01f, 1000u);

    CHECK(loop.wheel_target(0) > 0.0f);
    CHECK(loop.wheel_target(1) > 0.0f);
    for (uint8_t i = 2; i < 6; ++i) {
        CHECK(loop.wheel_target(i) == 0.0f);       // ← 装配层把脏尾巴堵住了（契约）
    }
    for (uint8_t i = 2; i < 6; ++i) {
        CHECK(last_pwm(i) == 0);                   // 未用到的轮子从没被下发
    }
}

int main() {
    test_inverse_hand_calc();
    test_acc_limit_property_and_tick_count();
    test_cmd_is_post_limit_and_limit_precedes_inverse();
    test_order_and_unused_wheels();
    test_dt_and_now_passthrough();
    test_round_trip();
    test_targets_actually_reach_wheels();
    test_omni_boundary_wheel_counts();
    test_actuator_seam();
    test_limit_result_exposed();
    test_unused_slots_are_zeroed_contract();

    if (g_fails == 0) { printf("ALL PASS\n"); return 0; }
    printf("%d FAILED\n", g_fails);
    return g_fails;
}
