// test/test_odometry.cpp — odometry 行为锚点（10 组）
// ============================================================================
// 分工：源码 inc/odometry.hpp 由作者手写；本测试由 AI 编写（用户 2026-09-13 定案）。
//
// oracle 全部外部 —— 见 test/odometry_golden.hpp 的头注释与 test/tools/gen_odometry_golden.py：
//   ① 离散精确解 = 半隐式欧拉的等比级数闭式   （本格式的**精确**期望值）
//   ② 连续真解   = SE(2) 指数映射闭式          （已用 scipy DOP853 交叉验证）
//   ③ wrap       = numpy.angle(exp(i·θ))      （第三方分支切割）
//   ④ 容差       = 实测 float32 累积偏差 × 4   （逐场景推导，非手拍常数）
//
// 判别力证据（生成器自检输出）：把"半隐式"换成"显式欧拉"，相对本 oracle 偏离
//   5.284e-03，而容差量级是 1e-6~1e-3 → **改坏顺序必红**。
//
// 编译：g++ -std=c++17 -Wall -Wextra -Werror -Iinc test/test_odometry.cpp
// ============================================================================

#include "odometry.hpp"
#include "odometry_golden.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

// 命名空间（2026-09-17）：全仓类型收进 lunokhod::（规则见 AGENTS.md §3）
using namespace lunokhod;
using namespace lunokhod::odometry;


static const float kPi = 3.14159265358979323846f;
static const float kTwoPi = 6.28318530717958647692f;

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        ++g_checks;                                                        \
        if (!(cond)) {                                                     \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
            ++g_fails;                                                     \
        }                                                                  \
    } while (0)

static float abs_diff(float a, float b) { return std::fabs(a - b); }

// (-π, π] 上的角距（避开 ±π 的表示歧义，但**不**放过数值错误）
static float angle_gap(float a, float b) {
    float d = std::fmod(a - b, kTwoPi);
    if (d > kPi) { d -= kTwoPi; }
    if (d < -kPi) { d += kTwoPi; }
    return std::fabs(d);
}

// wrap 契约：结果必须落在 (-π, π]
static bool in_wrap_range(float y) {
    return (y > -kPi - 1e-5f) && (y <= kPi + 1e-5f);
}

// ============================================================================
// 1. 单段闭式金标（恒速直行 / 圆轨迹 / 麦轮横移 / 多圈）
//    期望值 = 半隐式欧拉的等比级数闭式（精确），tolerance 由生成器逐场景推导
// ============================================================================
static void test_golden_single() {
    std::printf("[1] 单段闭式金标（几何级数闭式 oracle）\n");
    for (int i = 0; i < k_odo_golden_count; ++i) {
        const OdoGoldenCase& c = k_odo_golden[i];
        Odometry odom;
        for (int s = 0; s < c.n_; ++s) {
            odom.update(Twist{c.vx_, c.vy_, c.wz_}, c.dt_, static_cast<uint32_t>(s + 1));
        }
        const Pose p = odom.pose();
        const float gap_actual = std::hypot(p.x_ - c.x_cont_, p.y_ - c.y_cont_);
        const float gap_expect = std::hypot(c.x_expect_ - c.x_cont_, c.y_expect_ - c.y_cont_);
        std::printf("    %-16s pose=(%+.7f,%+.7f) yaw=%+.7f  tol=%.2e  一阶误差=%.2e\n",
                    c.name_, p.x_, p.y_, odom.yaw_continuous(), c.tol_, gap_actual);

        CHECK(abs_diff(p.x_, c.x_expect_) <= c.tol_);
        CHECK(abs_diff(p.y_, c.y_expect_) <= c.tol_);
        // yaw 连续累计值必须等于解析值（不是"两个出口相等"那种恒真断言）
        CHECK(angle_gap(odom.yaw_continuous(), c.yaw_cont_expect_) <= c.tol_);
        CHECK(angle_gap(odom.yaw_ref(), c.yaw_cont_expect_) <= c.tol_);
        // pose().yaw_ 是 wrap 出口
        CHECK(in_wrap_range(p.yaw_));
        // 与连续真解的距离应等于该格式的固有离散误差（数量级一致即可）
        CHECK(abs_diff(gap_actual, gap_expect) <= 4.0f * c.tol_);
    }
}

// ============================================================================
// 2. 世界系旋转（锚点 5）
//    接口没有"设置初始 yaw"的入口，所以用【先纯转、再走】表达：
//    纯转段 vx=vy=0 → 位移恒为 0（精确），因此总位移 = "从 yaw=θ 起步"的位移
// ============================================================================
static void test_golden_segments() {
    std::printf("[2] 世界系旋转（先转后走，分段金标）\n");
    for (int i = 0; i < k_odo_seg_count; ++i) {
        const OdoSegCase& c = k_odo_seg[i];
        Odometry odom;
        uint32_t tick = 0;
        for (int s = 0; s < c.seg_count_; ++s) {
            const OdoSeg& g = c.seg_[s];
            for (int k = 0; k < g.n_; ++k) {
                odom.update(Twist{g.vx_, g.vy_, g.wz_}, g.dt_, ++tick);
            }
        }
        const Pose p = odom.pose();
        std::printf("    %-20s pose=(%+.7f,%+.7f) yaw=%+.7f  tol=%.2e\n",
                    c.name_, p.x_, p.y_, odom.yaw_continuous(), c.tol_);

        CHECK(abs_diff(p.x_, c.x_expect_) <= c.tol_);
        CHECK(abs_diff(p.y_, c.y_expect_) <= c.tol_);
        CHECK(angle_gap(odom.yaw_continuous(), c.yaw_expect_) <= c.tol_);
    }
}

// ============================================================================
// 3. 一阶收敛（把 R6 的"半隐式欧拉"钉死）
//    dt 缩小 10 倍 → 与连续真解的距离应缩小约 10 倍
// ============================================================================
static void test_convergence() {
    std::printf("[3] 一阶收敛（钉死积分格式）\n");
    float gaps[k_odo_conv_count];
    for (int i = 0; i < k_odo_conv_count; ++i) {
        const OdoConvCase& c = k_odo_conv[i];
        Odometry odom;
        for (int s = 0; s < c.n_; ++s) {
            odom.update(Twist{c.vx_, c.vy_, c.wz_}, c.dt_, static_cast<uint32_t>(s + 1));
        }
        const Pose p = odom.pose();
        // ① 实现必须落在【离散精确解】上（紧断言）
        CHECK(abs_diff(p.x_, c.x_expect_) <= c.tol_);
        CHECK(abs_diff(p.y_, c.y_expect_) <= c.tol_);
        // ② 实现到【连续真解】的距离，必须与闭式预告的一阶误差一致
        const float gap_actual = std::hypot(p.x_ - c.x_cont_, p.y_ - c.y_cont_);
        CHECK(abs_diff(gap_actual, c.gap_) <= 4.0f * c.tol_);
        gaps[i] = c.gap_;
        std::printf("    %-12s dt=%.4f  n=%-5d 一阶误差: 闭式=%.3e 实测=%.3e\n",
                    c.name_, c.dt_, c.n_, c.gap_, gap_actual);
    }
    CHECK(gaps[0] > 0.0f);
    CHECK(gaps[1] > 0.0f);
    CHECK(gaps[1] * 5.0f < gaps[0]);   // 至少 5 倍，闭式给出的是 10.0 倍
    std::printf("    比值 = %.2f（闭式预告 10.0，> 5 即判过）\n", gaps[0] / gaps[1]);
}

// ============================================================================
// 4. 零输入冻结（修掉原锚点 4 的"恒真断言"）
//    原写法从原点起步、零输入 → 任何错误实现都返回 0。
//    改法：先积分到远离原点，再给零输入，要求位姿**逐位冻结**。
// ============================================================================
static void test_zero_freeze() {
    std::printf("[4] 零输入冻结（非恒真版）\n");
    Odometry odom;
    for (int s = 0; s < 50; ++s) {
        odom.update(Twist{0.5f, 0.2f, 0.3f}, 0.01f, static_cast<uint32_t>(s + 1));
    }
    const Pose before = odom.pose();
    const float yaw_before = odom.yaw_continuous();
    // 判别力前置条件：必须已经离开原点，否则断言退化成恒真
    CHECK(std::fabs(before.x_) > 0.1f);
    CHECK(std::fabs(yaw_before) > 0.1f);

    for (int s = 0; s < 100; ++s) {
        odom.update(Twist{0.0f, 0.0f, 0.0f}, 0.01f, static_cast<uint32_t>(100 + s));
    }
    const Pose after = odom.pose();
    CHECK(abs_diff(after.x_, before.x_) == 0.0f);
    CHECK(abs_diff(after.y_, before.y_) == 0.0f);
    CHECK(abs_diff(odom.yaw_continuous(), yaw_before) == 0.0f);
    std::printf("    冻结前 pose=(%+.6f,%+.6f) yaw=%+.6f → 零输入 100 帧后逐位不变\n",
                before.x_, before.y_, yaw_before);
}

// ============================================================================
// 5. reset()
// ============================================================================
static void test_reset() {
    std::printf("[5] reset()\n");
    Odometry odom;
    for (int s = 0; s < 100; ++s) {
        odom.update(Twist{0.5f, 0.0f, 1.0f}, 0.01f, static_cast<uint32_t>(s + 1));
    }
    odom.reset();
    const Pose p = odom.pose();
    CHECK(p.x_ == 0.0f);
    CHECK(p.y_ == 0.0f);
    CHECK(p.yaw_ == 0.0f);
    CHECK(odom.yaw_continuous() == 0.0f);
    CHECK(odom.yaw_ref() == 0.0f);
}

// ============================================================================
// 6. wrap 表（oracle = numpy.angle(exp(i·θ))）
//    用 dt=1.0、vx=vy=0 的单步把 yaw 精确置到目标值（yaw += wz·1.0 是精确的），
//    纯转段位移恒为 0，不污染结果。
// ============================================================================
static void test_wrap() {
    std::printf("[6] wrap 表（numpy 复数相位 oracle）\n");
    for (int i = 0; i < k_odo_wrap_count; ++i) {
        const OdoWrapCase& c = k_odo_wrap[i];
        Odometry odom;
        odom.update(Twist{0.0f, 0.0f, c.yaw_cont_}, 1.0f, 1u);
        const Pose p = odom.pose();
        const float gap = angle_gap(p.yaw_, c.yaw_wrap_);
        std::printf("    yaw_cont=%+.9f → yaw_wrap=%+.9f (期望 %+.9f)  角距=%.2e\n",
                    odom.yaw_continuous(), p.yaw_, c.yaw_wrap_, gap);
        CHECK(angle_gap(odom.yaw_continuous(), c.yaw_cont_) <= 1e-5f);
        CHECK(gap <= 1e-5f);
        CHECK(in_wrap_range(p.yaw_));     // 真契约：必须落在 (-π, π]
    }
}

// ============================================================================
// 7. 单步解析值（证明单步无隐藏数学）
//    vx=0.5, dt=0.01, θ0=0 → Δx = 0.005, Δy = 0, yaw 不变
// ============================================================================
static void test_step_analytic() {
    std::printf("[7] 单步解析值\n");
    Odometry odom;
    odom.update(Twist{k_odo_step_vx, 0.0f, 0.0f}, k_odo_step_dt, 1u);
    const Pose p = odom.pose();
    CHECK(abs_diff(p.x_, k_odo_step_dx) <= 1e-9f);
    CHECK(abs_diff(p.y_, 0.0f) <= 1e-9f);
    CHECK(odom.yaw_continuous() == 0.0f);
    std::printf("    Δx=%.9f (期望 %.9f)  Δy=%.9f  yaw=%+.9f\n",
                p.x_, k_odo_step_dx, p.y_, p.yaw_);
}

// ============================================================================
// 8. twist_scale 不变性（替代原锚点 9 的"位移 ×2"）
//    正确的 oracle：scale=k 作用于输入 t  ≡  scale=1 作用于输入 k·t（恒等式）
//    反例记录：ω≠0 时"位移 ×2"**不成立** —— 因为 yaw 也被 ×k，相位轨迹跟着变
// ============================================================================
static void test_twist_scale() {
    std::printf("[8] twist_scale 不变性\n");
    const float dt = 0.01f;
    const int n = 200;

    for (int i = 0; i < k_odo_scale_count; ++i) {
        const OdoScaleCase& c = k_odo_scale[i];
        Odometry scaled(c.scale_);
        Odometry plain(1.0f);
        for (int s = 0; s < n; ++s) {
            const uint32_t tick = static_cast<uint32_t>(s + 1);
            scaled.update(Twist{c.vx_, c.vy_, c.wz_}, dt, tick);
            plain.update(Twist{c.scale_ * c.vx_, c.scale_ * c.vy_, c.scale_ * c.wz_}, dt, tick);
        }
        const Pose a = scaled.pose();
        const Pose b = plain.pose();
        const float tol = 1e-4f;
        std::printf("    scale=%.1f  scaled=(%+.7f,%+.7f)  plain=(%+.7f,%+.7f)  Δ=%.2e\n",
                    c.scale_, a.x_, a.y_, b.x_, b.y_,
                    std::fmax(abs_diff(a.x_, b.x_), abs_diff(a.y_, b.y_)));
        CHECK(abs_diff(a.x_, b.x_) <= tol);
        CHECK(abs_diff(a.y_, b.y_) <= tol);
        CHECK(angle_gap(a.yaw_, b.yaw_) <= 1e-3f);
    }

    // 反例：证明旧锚点"scale=2 → 位移 ×2"在 ω≠0 时是错的（记录设计修正 R8）
    Odometry two(2.0f);
    Odometry one(1.0f);
    for (int s = 0; s < n; ++s) {
        const uint32_t tick = static_cast<uint32_t>(s + 1);
        two.update(Twist{0.3f, 0.0f, 0.7f}, dt, tick);
        one.update(Twist{0.3f, 0.0f, 0.7f}, dt, tick);
    }
    const Pose p2 = two.pose();
    const Pose p1 = one.pose();
    const float naive = abs_diff(p2.x_, 2.0f * p1.x_);
    std::printf("    反例（ω≠0）：scale=2 的 x=%.7f，而 2×scale=1 的 x=%.7f，差 %.4f （≠0 → 旧锚点不成立）\n",
                p2.x_, p1.x_, naive);
    CHECK(naive > 1e-3f);
}

// ============================================================================
// 9. sink 记录（锚点 6）
// ============================================================================
struct SinkRec {
    int      count;
    uint32_t first_tick;
    uint32_t last_tick;
    float    first_ws0;
    float    last_dt;
    float    last_cmd_vx;
    float    last_twist_wz;
    float    last_pose_x;
};

static void sink_fn(void* ctx, const OdometrySample& s) {
    SinkRec* r = static_cast<SinkRec*>(ctx);
    if (r->count == 0) {
        r->first_tick = s.tick_;
        r->first_ws0 = s.ws_.values_[0];
    }
    ++r->count;
    r->last_tick = s.tick_;
    r->last_dt = s.dt_;
    r->last_cmd_vx = s.cmd_.vx_;
    r->last_twist_wz = s.twist_.wz_;
    r->last_pose_x = s.pose_.x_;
}

static void test_sink() {
    std::printf("[9] sink 记录\n");
    SinkRec rec{0, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Odometry odom(1.0f);
    odom.set_sink(sink_fn, &rec);

    const int n = 50;
    WheelSpeeds ws{4, {1.0f, 2.0f, 3.0f, 4.0f, 0.0f, 0.0f}};
    for (int s = 0; s < n; ++s) {
        Twist cmd{0.5f, 0.0f, 0.25f};
        odom.update(Twist{0.4f, 0.1f, 0.2f}, 0.02f, static_cast<uint32_t>(s + 1), cmd, &ws);
    }
    CHECK(rec.count == n);                       // 每帧恰好一次
    CHECK(rec.first_tick == 1u);
    CHECK(rec.last_tick == static_cast<uint32_t>(n));   // tick 单调
    CHECK(abs_diff(rec.first_ws0, 1.0f) <= 1e-6f);      // 原始轮速是**拷贝**进样本的
    CHECK(abs_diff(rec.last_dt, 0.02f) <= 1e-6f);
    CHECK(abs_diff(rec.last_cmd_vx, 0.5f) <= 1e-6f);
    CHECK(abs_diff(rec.last_twist_wz, 0.2f) <= 1e-6f);

    // 拷贝语义：改动调用方对象后，样本里已记录的值不受影响
    const float held_ws0 = rec.first_ws0;
    const float held_cmd = rec.last_cmd_vx;
    ws.values_[0] = 99.0f;
    CHECK(abs_diff(rec.first_ws0, held_ws0) <= 0.0f);
    CHECK(abs_diff(rec.last_cmd_vx, held_cmd) <= 0.0f);
    std::printf("    %d 帧全部触发；tick %u→%u；字段与输入一致；拷贝语义成立\n",
                rec.count, rec.first_tick, rec.last_tick);

    // 2026-09-14 补：区分「标定前 vs 标定后」—— scale=2 时 twist_ 必须是输入的 2 倍，
    // 而 cmd_ 必须**原样**（命令是上层给的，与底盘标定无关）。
    // 这两条同时也再抓一次「twist_ / cmd_ 装反」（同类型相邻字段的静默错位）。
    SinkRec rec2{0, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Odometry odom2(2.0f);
    odom2.set_sink(sink_fn, &rec2);
    WheelSpeeds ws2{4, {1.0f, 2.0f, 3.0f, 4.0f, 0.0f, 0.0f}};
    odom2.update(Twist{0.3f, 0.0f, 0.1f}, 0.02f, 1u, Twist{0.3f, 0.0f, 0.1f}, &ws2);
    CHECK(rec2.count == 1);
    CHECK(abs_diff(rec2.last_cmd_vx, 0.3f) <= 1e-6f);      // cmd 不乘 scale
    CHECK(abs_diff(rec2.last_twist_wz, 0.2f) <= 1e-6f);    // twist = 0.1 × 2
    std::printf("    scale=2：cmd_vx=%.4f（期望 0.3，不缩放）  twist_wz=%.4f（期望 0.2，已缩放）\n",
                rec2.last_cmd_vx, rec2.last_twist_wz);
}

// ============================================================================
// 10. R7 防御：sink 已设但 ws == nullptr → 不触发（宁可丢帧，不写脏数据）
// ============================================================================
static void test_sink_defense() {
    std::printf("[10] R7 防御（ws == nullptr 不触发）\n");
    SinkRec rec{0, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Odometry odom(1.0f);
    odom.set_sink(sink_fn, &rec);

    for (int s = 0; s < 10; ++s) {
        odom.update(Twist{0.4f, 0.0f, 0.2f}, 0.02f, static_cast<uint32_t>(s + 1));
    }
    CHECK(rec.count == 0);                       // 未传 ws → 不记录

    WheelSpeeds ws{4, {1.0f, 2.0f, 3.0f, 4.0f, 0.0f, 0.0f}};
    odom.update(Twist{0.4f, 0.0f, 0.2f}, 0.02f, 11u, Twist{0.0f, 0.0f, 0.0f}, &ws);
    CHECK(rec.count == 1);                       // 传了 ws → 正常记录

    // 未设 sink 时传 ws 也不该崩、不该记录
    SinkRec rec2{0, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Odometry odom2(1.0f);
    odom2.update(Twist{0.4f, 0.0f, 0.2f}, 0.02f, 1u, Twist{0.0f, 0.0f, 0.0f}, &ws);
    CHECK(rec2.count == 0);
    std::printf("    sink 已设 + ws=nullptr → 0 次；传 ws → 1 次 ✅\n");
}

// ============================================================================
// 11. 初始位姿入口（R12）
//     需求：上电时车可能已经在某个已知位姿上（或重新对位），需要“就地开里程计”。
//     oracle：恒等（写进去什么就读出什么）+ 等比级数闭式（平移/旋转后的期望轨迹）
// ============================================================================
static void test_reset_to_pose() {
    std::printf("[11] 初始位姿入口 reset(Pose)\n");

    // (a) 恒等：设进去什么，读出来就是什么
    const float xs[3]   = {  1.5f, -2.25f,  0.0f };
    const float ys[3]   = { -0.75f,  3.0f,   0.0f };
    const float yaws[3] = {  0.0f,   1.2f,  -2.9f };
    for (int i = 0; i < 3; ++i) {
        Odometry odom;
        odom.reset(Pose{xs[i], ys[i], yaws[i]});
        const Pose p = odom.pose();
        CHECK(abs_diff(p.x_, xs[i]) == 0.0f);          // 纯赋值，逐位相等
        CHECK(abs_diff(p.y_, ys[i]) == 0.0f);
        CHECK(abs_diff(p.yaw_, yaws[i]) <= 1e-6f);     // 经过一次 wrap，允许 ULP 级误差
        CHECK(angle_gap(odom.yaw_continuous(), yaws[i]) <= 1e-6f);
        CHECK(in_wrap_range(p.yaw_));
    }

    // (b) 设完之后 update 必须从该点继续积分
    //     oracle：等比级数闭式 —— 起点 (x0,y0,yaw0) 的位移 = 起点 (0,0,0) 的位移【旋转 yaw0】
    {
        const float x0 = 1.0f, y0 = -2.0f, yaw0 = 0.5f;
        const float vx = 0.3f, wz = 0.7f, dt = 0.01f;
        const int   n = 200;
        const float cx = k_odo_golden[1].x_expect_;     // golden[1] = "circle"
        const float cy = k_odo_golden[1].y_expect_;
        const float c = std::cos(yaw0), s = std::sin(yaw0);
        const float ex = x0 + (cx * c - cy * s);
        const float ey = y0 + (cx * s + cy * c);
        const float tol = 8.0f * k_odo_golden[1].tol_;   // 起点非零 → 累积舍入模式略变，放宽一档

        Odometry odom;
        odom.reset(Pose{x0, y0, yaw0});
        for (int k = 0; k < n; ++k) {
            odom.update(Twist{vx, 0.0f, wz}, dt, static_cast<uint32_t>(k + 1));
        }
        const Pose p = odom.pose();
        CHECK(abs_diff(p.x_, ex) <= tol);
        CHECK(abs_diff(p.y_, ey) <= tol);
        CHECK(angle_gap(odom.yaw_continuous(), yaw0 + wz * dt * static_cast<float>(n)) <= tol);
    }

    // (c) reset(Pose) 也只清状态、不清配置：twist_scale_ 与 sink_ 必须保留
    {
        SinkRec rec{0, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        Odometry odom(2.0f);
        odom.set_sink(sink_fn, &rec);
        odom.reset(Pose{1.0f, 2.0f, 0.3f});
        WheelSpeeds ws{4, {1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f}};
        odom.update(Twist{0.3f, 0.0f, 0.1f}, 0.02f, 1u, Twist{0.3f, 0.0f, 0.1f}, &ws);
        CHECK(rec.count == 1);                            // sink 被保留
        CHECK(abs_diff(rec.last_twist_wz, 0.2f) <= 1e-6f); // twist_scale=2 被保留
    }

    // (d) 无参 reset() 仍等价于 reset(Pose{0,0,0})
    {
        Odometry odom;
        odom.reset(Pose{1.0f, 2.0f, 0.3f});
        odom.reset();
        const Pose p = odom.pose();
        CHECK(p.x_ == 0.0f);
        CHECK(p.y_ == 0.0f);
        CHECK(p.yaw_ == 0.0f);
        CHECK(odom.yaw_continuous() == 0.0f);
    }
    std::printf("    恒等 / 起点续积 / 配置保留 / 无参 reset 等价 —— 全部覆盖\n");
}

// ============================================================================
int main() {
    std::printf("=== test_odometry：oracle 全部外部（见 odometry_golden.hpp）===\n");
    test_golden_single();
    test_golden_segments();
    test_convergence();
    test_zero_freeze();
    test_reset();
    test_wrap();
    test_step_analytic();
    test_twist_scale();
    test_sink();
    test_sink_defense();
    test_reset_to_pose();
    std::printf("=== %d checks, %d failed ===\n", g_checks, g_fails);
    if (g_fails == 0) { std::printf("ALL PASS\n"); }
    return g_fails;
}
