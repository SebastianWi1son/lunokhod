// 锚点测试 T1~T6 + 防御 + 回跳回归（L1 修复验证）
// 编译: g++ -std=c++17 -Wall -Wextra -Werror -Iinc test/test_twist_acc_limiter.cpp src/twist_acc_limiter.cpp -o /tmp/tal && /tmp/tal
#include "twist_acc_limiter.hpp"
#include <cstdio>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)
static bool close(float a, float b) { return (a - b < 1e-5f) && (b - a < 1e-5f); }

static void test_t1_ramp_up() {
    // 0 → 0.5, acc=1, dt=0.1 → 0.1/0.2/0.3/0.4/0.5
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    float expect = 0.0f;
    for (int i = 0; i < 4; ++i) {            // 前 4 帧：0.1/0.2/0.3/0.4 饱和
        auto r = lim.limit({0.5f, 0.0f, 0.0f}, 0.1f);
        expect += 0.1f;
        CHECK(close(r.out_.vx_, expect));
        CHECK(r.is_vx_lim_);
    }
    auto r5 = lim.limit({0.5f, 0.0f, 0.0f}, 0.1f);   // 第 5 帧：0.5 到达（区间含端点）
    CHECK(close(r5.out_.vx_, 0.5f));
    CHECK(!r5.is_vx_lim_);                    // 到达后不再饱和
}

static void test_t2_ramp_down() {
    // 先到 0.5，再 cmd=0 → 0.4/0.3/0.2/0.1/0
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    for (int i = 0; i < 5; ++i) lim.limit({0.5f, 0.0f, 0.0f}, 0.1f);
    float expect = 0.5f;
    for (int i = 0; i < 5; ++i) {
        auto r = lim.limit({0.0f, 0.0f, 0.0f}, 0.1f);
        expect -= 0.1f;
        CHECK(close(r.out_.vx_, expect));
    }
    auto r = lim.limit({0.0f, 0.0f, 0.0f}, 0.1f);
    CHECK(close(r.out_.vx_, 0.0f));          // 保持 0
}

static void test_t3_reverse() {
    // 方向反转：0.3 → -0.3 → 0.3/0.2/0.1/0/-0.1/-0.2/-0.3（过零平滑）
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    for (int i = 0; i < 3; ++i) lim.limit({0.3f, 0.0f, 0.0f}, 0.1f);
    float expect = 0.3f;
    for (int i = 0; i < 6; ++i) {
        auto r = lim.limit({-0.3f, 0.0f, 0.0f}, 0.1f);
        expect -= 0.1f;
        CHECK(close(r.out_.vx_, expect));
    }
}

static void test_t4_follow() {
    // 未饱和跟踪：0.1 → 0.15（变化 0.05 < max_step 0.1）→ 一步到位
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    auto r1 = lim.limit({0.1f, 0.0f, 0.0f}, 0.1f);
    CHECK(close(r1.out_.vx_, 0.1f));
    auto r2 = lim.limit({0.15f, 0.0f, 0.0f}, 0.1f);
    CHECK(close(r2.out_.vx_, 0.15f));
    CHECK(!r2.is_vx_lim_);
}

static void test_t5_channel_iso() {
    // 通道独立 + 上报：先稳定 vy/wz，再 vx 大幅跳变 → vx 饱和、vy/wz 不动
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    for (int i = 0; i < 5; ++i) lim.limit({0.0f, 0.3f, 0.2f}, 0.1f);  // vy/wz 稳定
    auto r = lim.limit({0.5f, 0.3f, 0.2f}, 0.1f);
    CHECK(close(r.out_.vx_, 0.1f));          // vx 被限：只走 0.1
    CHECK(close(r.out_.vy_, 0.3f));          // vy 不动
    CHECK(close(r.out_.wz_, 0.2f));          // wz 不动
    CHECK(r.is_vx_lim_ && !r.is_vy_lim_ && !r.is_wz_lim_);
}

static void test_t6_hold() {
    // 保持：目标不变 → 输出不变（稳态无漂）
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    lim.limit({0.2f, 0.0f, 0.0f}, 0.1f);     // 0.1
    lim.limit({0.2f, 0.0f, 0.0f}, 0.1f);     // 0.2
    auto r2 = lim.limit({0.2f, 0.0f, 0.0f}, 0.1f);
    auto r3 = lim.limit({0.2f, 0.0f, 0.0f}, 0.1f);
    CHECK(close(r2.out_.vx_, 0.2f) && close(r3.out_.vx_, 0.2f));
}

static void test_defense() {
    // 防御：dt<=0 直通 + 状态同步（L1 回归：下一帧不回跳）
    TwistAccLimiter lim(1.0f, 1.0f, 1.0f);
    for (int i = 0; i < 5; ++i) lim.limit({0.5f, 0.0f, 0.0f}, 0.1f);  // 到 0.5
    auto bad = lim.limit({0.3f, 0.0f, 0.0f}, 0.0f);                   // 非法 dt
    CHECK(close(bad.out_.vx_, 0.3f));        // 直通
    CHECK(!bad.is_vx_lim_);
    auto good = lim.limit({0.3f, 0.0f, 0.0f}, 0.1f);                  // dt 恢复
    CHECK(close(good.out_.vx_, 0.3f));       // 不回跳
    CHECK(!good.is_vx_lim_);
}

int main() {
    test_t1_ramp_up();
    test_t2_ramp_down();
    test_t3_reverse();
    test_t4_follow();
    test_t5_channel_iso();
    test_t6_hold();
    test_defense();
    if (fails == 0) printf("ALL PASS ✅\n");
    else printf("%d FAILS ❌\n", fails);
    return fails == 0 ? 0 : 1;
}