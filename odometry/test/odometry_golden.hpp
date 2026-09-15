// ⚠ 自动生成，请勿手改 —— 由 test/tools/gen_odometry_golden.py 生成
//
// 期望值来源（全部外部，无一行自造）：
//   x_expect_/y_expect_ : 半隐式欧拉的【等比级数闭式】= 本离散格式的精确解
//   x_cont_/y_cont_     : 【SE(2) 指数映射】= 连续真解（已用 scipy DOP853 交叉验证）
//   yaw_wrap_           : numpy.angle(exp(iθ))，第三方分支切割
//   tol_                : 实测 float32 累积偏差 × 4（逐场景，非手拍常数）
// 自检：闭式 vs float64 循环 2e-14；连续闭式 vs scipy 9e-15；
//       显式欧拉相对本 oracle 偏离 ~5e-3 ≫ tol → oracle 能区分积分格式。
#pragma once
#include <cstdint>

struct OdoSeg { float vx_, vy_, wz_, dt_; int n_; };

// ---- 单段场景（起点恒为原点 / yaw=0）----
struct OdoGoldenCase {
    const char* name_;
    float vx_, vy_, wz_, dt_;
    int   n_;
    float x_expect_, y_expect_;
    float x_cont_, y_cont_;
    float yaw_cont_expect_;
    float tol_;
};
static const OdoGoldenCase k_odo_golden[] = {
    {"straight", 0.5f, 0.0f, 0.0f, 0.01f, 100, 0.5f, 0.0f, 0.5f, 0.0f, 0.0f, 1.31130219e-06f},
    {"circle", 0.3f, 0.0f, 0.7f, 0.01f, 200, 0.421088825f, 0.357205089f, 0.422335599f, 0.355728367f, 1.4f, 1.44468021e-06f},
    {"mecanum_slide", 0.0f, 0.4f, 0.5f, 0.01f, 300, -0.74540368f, 0.796135801f, -0.743410239f, 0.797995989f, 1.5f, 4.76837158e-06f},
    {"multiturn", 0.2f, 0.0f, 3.0f, 0.01f, 1000, -0.0667095833f, 0.0553909762f, -0.0658687749f, 0.0563832367f, 30.0f, 0.00109100342f},
};
static const int k_odo_golden_count = 4;

// ---- 多段场景：用于「世界系旋转」锚点（先纯转、再走；纯转段位移恒为 0，故精确）----
struct OdoSegCase {
    const char* name_;
    OdoSeg seg_[3];
    int   seg_count_;
    float x_expect_, y_expect_, yaw_expect_, tol_;
};
static const OdoSegCase k_odo_seg[] = {
    {"rot90_then_straight", {{0.0f, 0.0f, 1.0f, 0.01f, 157}, {0.5f, 0.0f, 0.0f, 0.01f, 100}, {0.0f, 0.0f, 0.0f, 0.0f, 0}}, 2, 0.000398163355f, 0.499999841f, 1.57f, 4.55856323e-06f},
    {"circle_after_turn", {{0.0f, 0.0f, 1.0f, 0.01f, 157}, {0.3f, 0.0f, 0.7f, 0.01f, 200}, {0.0f, 0.0f, 0.0f, 0.0f, 0}}, 2, -0.356869652f, 0.421373143f, 2.97f, 2.84957886e-05f},
    {"slide_after_turn", {{0.0f, 0.0f, -1.0f, 0.01f, 100}, {0.0f, 0.4f, 0.5f, 0.01f, 300}, {0.0f, 0.0f, 0.0f, 0.0f, 0}}, 2, 0.26718185f, 1.05738958f, 0.5f, 2.06422722e-06f},
};
static const int k_odo_seg_count = 3;

// ---- 收敛性：相同 T、不同 dt。gap = |离散 − 连续|（一阶格式应随 dt 线性缩小）----
struct OdoConvCase {
    const char* name_;
    float vx_, vy_, wz_, dt_;
    int   n_;
    float x_expect_, y_expect_;   // 离散精确解
    float x_cont_, y_cont_;       // 连续真解
    float gap_;                   // |离散 − 连续|，期望的一阶误差
    float tol_;
};
static const OdoConvCase k_odo_conv[] = {
    {"conv_dt010", 0.3f, 0.0f, 0.7f, 0.01f, 200, 0.421088825f, 0.357205089f, 0.422335599f, 0.355728367f, 0.00193265438f, 1.44468021e-06f},
    {"conv_dt001", 0.3f, 0.0f, 0.7f, 0.001f, 2000, 0.422211076f, 0.35587617f, 0.422335599f, 0.355728367f, 0.000193265308f, 2.2983551e-05f},
};
static const int k_odo_conv_count = 2;

// ---- twist_scale 不变性：scale=k 作用于输入 t  ≡  scale=1 作用于输入 k·t ----
//   ⚠ 设计 §8 锚点 9 原写「位移 ×2」只对 ω=0 成立；ω≠0 时位移不是线性缩放（见 R8）
struct OdoScaleCase { float scale_, vx_, vy_, wz_; };
static const OdoScaleCase k_odo_scale[] = {
    {2.0f, 0.3f, 0.0f, 0.7f},
    {0.5f, 0.3f, 0.0f, 0.7f},
    {2.0f, 0.0f, 0.4f, 0.5f},
    {3.0f, 0.5f, 0.0f, 0.0f},
    {2.0f, 0.3f, 0.0f, 0.0f},
};
static const int k_odo_scale_count = 5;

// ---- wrap 表：借 numpy.angle(exp(iθ)) ----
struct OdoWrapCase { float yaw_cont_, yaw_wrap_; };
static const OdoWrapCase k_odo_wrap[] = {
    {0.0f, 0.0f},
    {1.57079633f, 1.57079633f},
    {3.14159265f, 3.14159265f},
    {-3.14159265f, -3.14159265f},
    {4.71238898f, -1.57079633f},
    {6.28318531f, -2.4492936e-16f},
    {23.5619449f, -1.57079633f},
    {-23.5619449f, 1.57079633f},
    {30.0f, -1.41592654f},
    {-0.001f, -0.001f},
};
static const int k_odo_wrap_count = 10;

// ---- 单步解析值：直行 vx=0.5, dt=0.01, θ0=0 → 每步 Δx = 0.005（无累积误差）----
static const float k_odo_step_vx = 0.5f;
static const float k_odo_step_dt = 0.01f;
static const float k_odo_step_dx = 0.005f;
