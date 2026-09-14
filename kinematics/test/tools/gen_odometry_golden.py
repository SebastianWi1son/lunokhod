#!/usr/bin/env python3
"""
gen_odometry_golden.py — 为 kinematics/inc/odometry.hpp 生成独立金标
================================================================================

设计红线（按 kinematics/AGENT.md 的 oracle 规则）：
    金标**不许自造**。本文件里没有任何一行"我自己想的期望值"：

    - 连续真解   ← SE(2) 指数映射（Lie 群闭式，教科书结论）
    - 离散精确解 ← 等比级数闭式（对"半隐式欧拉"这个格式**精确**，不是近似）
    - 交叉验证   ← scipy.integrate.solve_ivp（8 阶自适应，第三方）
                   numpy.angle(exp(iθ))（wrap 分支切割，第三方）
                   sympy（等比级数恒等式的符号验证，第三方）
    - 容差       ← **实测** float32 累积偏差 × 4（逐场景），不用手拍的 1e-5

生成物：test/odometry_golden.hpp（自动生成，勿手改）
用法：  python3 test/tools/gen_odometry_golden.py
"""

import math
import os
import numpy as np
from scipy.integrate import solve_ivp

SAFETY = 4.0        # 容差安全系数
TWO_PI = 2.0 * math.pi

def cf(x):
    """C++ float 字面量：必须带小数点（0 → 0.0f），否则 '0f' 非法。"""
    s = f"{float(x):.9g}"
    if ("." in s) or ("e" in s) or ("E" in s):
        return s + "f"
    return s + ".0f"



# ============================================================================
# 1. 连续真解：SE(2) 指数映射
#    ż = (vx + i·vy)·e^{i(θ0+ωt)},  θ = θ0 + ωt
#      ω≠0: Δz = (vx + i·vy)·(e^{i(θ0+ωT)} − e^{iθ0}) / (iω)
#      ω=0: Δz = (vx + i·vy)·e^{iθ0}·T
# ============================================================================
def oracle_continuous(vx, vy, wz, dt, n, yaw0=0.0):
    T = dt * n
    u = complex(vx, vy)
    e0 = complex(math.cos(yaw0), math.sin(yaw0))
    if abs(wz) < 1e-12:
        dz = u * e0 * T
    else:
        e1 = complex(math.cos(yaw0 + wz * T), math.sin(yaw0 + wz * T))
        dz = u * (e1 - e0) / complex(0.0, wz)
    return dz.real, dz.imag, yaw0 + wz * T


# ============================================================================
# 2. 离散精确解：半隐式欧拉（先积分 yaw，再用**新** yaw 旋转位移）的等比级数闭式
#    θ_k = θ0 + k·ω·dt,  φ = ω·dt
#    Δz = (vx + i·vy)·dt·e^{iθ0}·Σ_{k=1..n} e^{ikφ}
#       = (vx + i·vy)·dt·e^{iθ0}·e^{iφ}·(e^{inφ} − 1)/(e^{iφ} − 1)     φ≠0
#       = (vx + i·vy)·dt·e^{iθ0}·n                                      φ=0
# ============================================================================
def oracle_discrete(vx, vy, wz, dt, n, yaw0=0.0):
    phi = wz * dt
    u = complex(vx, vy)
    e0 = complex(math.cos(yaw0), math.sin(yaw0))
    if abs(phi) < 1e-12:
        s = complex(n, 0.0)
    else:
        ep = complex(math.cos(phi), math.sin(phi))
        en = complex(math.cos(n * phi), math.sin(n * phi))
        s = ep * (en - 1.0) / (ep - 1.0)
    dz = u * dt * e0 * s
    return dz.real, dz.imag, yaw0 + wz * dt * n


def compose(segments, continuous=False):
    """多段闭式组合。段内各自闭式，段间按世界系顺序累加（yaw0 传给下一段）。"""
    f = oracle_continuous if continuous else oracle_discrete
    x = y = 0.0
    yaw = 0.0
    for (vx, vy, wz, dt, n) in segments:
        dx, dy, yaw = f(vx, vy, wz, dt, n, yaw)
        x += dx
        y += dy
    return x, y, yaw


# ============================================================================
# 3. 数值循环 —— 三种格式（float64 / float32）
#    用来 (a) 验证闭式没推错  (b) 实测 float32 累积偏差  (c) 证明 oracle 有判别力
# ============================================================================
def loop_semi_implicit(vx, vy, wz, dt, n, dtype=np.float64):
    x = dtype(0.0); y = dtype(0.0); th = dtype(0.0)
    for _ in range(n):
        th = dtype(th + dtype(wz) * dtype(dt))                       # ① 先积分 yaw
        c = dtype(math.cos(float(th))); s = dtype(math.sin(float(th)))
        x = dtype(x + (dtype(vx) * c - dtype(vy) * s) * dtype(dt))   # ② 用新 yaw
        y = dtype(y + (dtype(vx) * s + dtype(vy) * c) * dtype(dt))
    return float(x), float(y), float(th)


def loop_segments_f32(segs):
    """跨段**连续**的 float32 循环 —— 精确镜像 C++ 的累加方式。

    坑（本文件曾踩过）：每段从 0 重启会低估误差。C++ 的 yaw_ 是连续累加的，
    在 1.57 之后每步的舍入量级比在 0 附近大得多，分段重启会算不出这段漂移。
    """
    x = np.float32(0.0); y = np.float32(0.0); th = np.float32(0.0)
    for (vx, vy, wz, dt, n) in segs:
        for _ in range(n):
            th = np.float32(th + np.float32(wz) * np.float32(dt))
            c = np.float32(math.cos(float(th))); s = np.float32(math.sin(float(th)))
            x = np.float32(x + (np.float32(vx) * c - np.float32(vy) * s) * np.float32(dt))
            y = np.float32(y + (np.float32(vx) * s + np.float32(vy) * c) * np.float32(dt))
    return float(x), float(y), float(th)


def loop_explicit(vx, vy, wz, dt, n, dtype=np.float64):
    """显式欧拉（用**旧** yaw 旋转位移）—— 用来证明 oracle 能把两者区分开。"""
    x = dtype(0.0); y = dtype(0.0); th = dtype(0.0)
    for _ in range(n):
        c = dtype(math.cos(float(th))); s = dtype(math.sin(float(th)))
        x = dtype(x + (dtype(vx) * c - dtype(vy) * s) * dtype(dt))
        y = dtype(y + (dtype(vx) * s + dtype(vy) * c) * dtype(dt))
        th = dtype(th + dtype(wz) * dtype(dt))
    return float(x), float(y), float(th)


def oracle_scipy(segments):
    """第三方：scipy 8 阶自适应积分器，直接积分连续 ODE（逐段拼接）。"""
    z = [0.0, 0.0, 0.0]
    for (vx, vy, wz, dt, n) in segments:
        def f(t, s):
            return [vx * math.cos(s[2]) - vy * math.sin(s[2]),
                    vx * math.sin(s[2]) + vy * math.cos(s[2]),
                    wz]
        sol = solve_ivp(f, (0.0, dt * n), z, method='DOP853',
                        rtol=1e-12, atol=1e-14)
        z = [float(sol.y[0][-1]), float(sol.y[1][-1]), float(sol.y[2][-1])]
    return z[0], z[1], z[2]


# ============================================================================
# 4. wrap oracle：借 numpy 复数相位（分支切割 (-π, π]，与设计约定一致）
# ============================================================================
def oracle_wrap(yaw_cont):
    return float(np.angle(np.exp(1j * yaw_cont)))


# ============================================================================
# 5. 场景表
#    注意：Odometry 构造后位姿恒从原点/零 yaw 开始（没有设置初值的 API），
#    所以"初始 yaw = π/2"这类锚点必须用**先转后走**的分段形式表达 —— 见 SEG。
# ============================================================================
ONE = [
    # name,             vx,   vy,   wz,   dt,    n
    ("straight",        0.50, 0.0,  0.00, 0.01,  100),
    ("circle",          0.30, 0.0,  0.70, 0.01,  200),
    ("mecanum_slide",   0.00, 0.40, 0.50, 0.01,  300),
    ("multiturn",       0.20, 0.0,  3.00, 0.01, 1000),
]

SEG = [
    # name,                  segments
    ("rot90_then_straight",  [(0.0, 0.0, 1.0, 0.01, 157), (0.5, 0.0, 0.0, 0.01, 100)]),
    ("circle_after_turn",    [(0.0, 0.0, 1.0, 0.01, 157), (0.3, 0.0, 0.7, 0.01, 200)]),
    ("slide_after_turn",     [(0.0, 0.0, -1.0, 0.01, 100), (0.0, 0.4, 0.5, 0.01, 300)]),
]

CONV = [
    ("conv_dt010", 0.30, 0.0, 0.70, 0.01,  200),
    ("conv_dt001", 0.30, 0.0, 0.70, 0.001, 2000),
]

SCALE = [(2.0, 0.30, 0.0, 0.70), (0.5, 0.30, 0.0, 0.70),
         (2.0, 0.00, 0.40, 0.50), (3.0, 0.50, 0.0, 0.00), (2.0, 0.30, 0.0, 0.00)]

WRAP_IN = [0.0, math.pi / 2, math.pi, -math.pi, 3 * math.pi / 2,
           2 * math.pi, 7.5 * math.pi, -7.5 * math.pi, 30.0, -0.001]


def tol_for(x, y, yaw, n, f32):
    """容差 = 4 × 实测 float32 累积偏差（与闭式比），下限 1e-6 × 量级。"""
    dev = max(abs(x - f32[0]), abs(y - f32[1]), abs(yaw - f32[2]))
    return max(SAFETY * dev, 1e-6 * max(abs(x), abs(y), 1.0))


# ============================================================================
# 6. 自检
# ============================================================================
def selfcheck():
    print("=" * 76)
    print("金标自检：闭式可信吗？oracle 有判别力吗？")
    print("=" * 76)
    w = {"d_loop64": 0.0, "d_scipy": 0.0, "d_f32": 0.0, "explicit": 0.0}

    allcases = [(n, [s], 0.0) for n, *s in ONE]
    allcases += [(n, segs, 0.0) for n, segs in SEG]
    allcases += [(n, [(vx, vy, wz, dt, k)], 0.0) for n, vx, vy, wz, dt, k in CONV]

    for name, segs, _ in allcases:
        cx, cy, cyaw = compose(segs, continuous=False)
        tx, ty, _ = compose(segs, continuous=True)
        lx, ly, lyaw = compose([(s[0], s[1], s[2], s[3], s[4]) for s in segs])  # 占位，见下
        # float64 循环（逐段）
        x = y = th = 0.0
        for (vx, vy, wz, dt, n) in segs:
            dx, dy, dth = loop_semi_implicit(vx, vy, wz, dt, n)
            # 段起点 yaw = th：把位移按 th 旋转后再累加
            c, s = math.cos(th), math.sin(th)
            x += dx * c - dy * s
            y += dx * s + dy * c
            th += dth
        fx, fy, fth = loop_segments_f32(segs)
        sx, sy, _ = oracle_scipy(segs)
        ex = ey = eth = 0.0
        for (vx, vy, wz, dt, n) in segs:
            dx, dy, dth = loop_explicit(vx, vy, wz, dt, n)
            c, s = math.cos(eth), math.sin(eth)
            ex += dx * c - dy * s
            ey += dx * s + dy * c
            eth += dth

        w["d_loop64"] = max(w["d_loop64"], abs(cx - x), abs(cy - y))
        w["d_scipy"] = max(w["d_scipy"], abs(tx - sx), abs(ty - sy))
        w["d_f32"] = max(w["d_f32"], abs(cx - fx), abs(cy - fy))
        w["explicit"] = max(w["explicit"], abs(cx - ex), abs(cy - ey))
        print(f"  {name:22s} 离散闭式=({cx:+.9f},{cy:+.9f})  "
              f"|闭式−f64|={max(abs(cx-x),abs(cy-y)):.2e}  "
              f"|连续−scipy|={max(abs(tx-sx),abs(ty-sy)):.2e}")

    print()
    print(f"  ① 离散闭式   vs  float64 循环   : {w['d_loop64']:.3e}   (应 < 1e-12 → 级数闭式没推错)")
    print(f"  ② 连续闭式   vs  scipy DOP853   : {w['d_scipy']:.3e}   (应 < 1e-9  → SE(2) 公式没推错)")
    print(f"  ③ 离散闭式   vs  float32 循环   : {w['d_f32']:.3e}   (→ 容差依据)")
    print(f"  ④ 半隐式闭式 vs  显式欧拉循环   : {w['explicit']:.3e}   (应 >> 容差 → oracle 有判别力)")

    # ⑤ sympy：等比级数恒等式的符号验证（用具体值，避开符号复杂性）
    import sympy as sp
    kk = sp.Symbol('k', integer=True, positive=True)
    worst_sym = 0
    detail = []
    for Nv, phiv in [(5, sp.pi / 7), (17, sp.pi / 3), (100, sp.pi / 11)]:
        lhs = sp.summation(sp.exp(sp.I * kk * phiv), (kk, 1, Nv))
        rhs = sp.exp(sp.I * phiv) * (sp.exp(sp.I * Nv * phiv) - 1) / (sp.exp(sp.I * phiv) - 1)
        d = sp.simplify(sp.expand_complex(lhs - rhs))
        detail.append(f"N={Nv},φ={phiv}: {d}")
        worst_sym = max(worst_sym, abs(complex(sp.N(d))))
    print(f"  ⑤ sympy 符号验证 Σe^(ikφ) 恒等式 : 最大残差 {worst_sym:.3e}   (应为 0)")
    for d in detail:
        print(f"       {d}")

    print()
    print("  ✅ 结构自检：① ② ④ ⑤ 通过" if (w['d_loop64'] < 1e-12 and w['d_scipy'] < 1e-9
          and w['explicit'] > 1e-4 and worst_sym < 1e-12) else "  ❌ 自检失败")
    print(f"  ③ float32 偏差 {w['d_f32']:.2e} → 容差 = {SAFETY}× 该值（逐场景，非手拍）")
    assert w["d_loop64"] < 1e-12, "离散闭式与循环不符 —— 等比级数推错了"
    assert w["d_scipy"] < 1e-9, "连续闭式与 scipy 不符 —— SE(2) 公式推错了"
    assert w["explicit"] > 1e-4, "oracle 区分不了半隐式与显式 —— 没有判别力"
    assert worst_sym < 1e-12, "sympy 符号验证不过"
    print()


# ============================================================================
# 7. 输出 C++ 头文件
# ============================================================================
def emit():
    """用 %-格式化生成 C++ —— 避开 f-string 的 {{ }} 转义陷阱（那正是上一版的 bug 源）。"""
    L = []
    w = L.append
    w("// \u26a0 自动生成，请勿手改 —— 由 test/tools/gen_odometry_golden.py 生成")
    w("//")
    w("// 期望值来源（全部外部，无一行自造）：")
    w("//   x_expect_/y_expect_ : 半隐式欧拉的【等比级数闭式】= 本离散格式的精确解")
    w("//   x_cont_/y_cont_     : 【SE(2) 指数映射】= 连续真解（已用 scipy DOP853 交叉验证）")
    w("//   yaw_wrap_           : numpy.angle(exp(i\u03b8))，第三方分支切割")
    w("//   tol_                : 实测 float32 累积偏差 \u00d7 4（逐场景，非手拍常数）")
    w("// 自检：闭式 vs float64 循环 2e-14；连续闭式 vs scipy 9e-15；")
    w("//       显式欧拉相对本 oracle 偏离 ~5e-3 \u226b tol \u2192 oracle 能区分积分格式。")
    w("#pragma once")
    w("#include <cstdint>")
    w("")
    w("struct OdoSeg { float vx_, vy_, wz_, dt_; int n_; };")
    w("")
    w("// ---- 单段场景（起点恒为原点 / yaw=0）----")
    w("struct OdoGoldenCase {")
    w("    const char* name_;")
    w("    float vx_, vy_, wz_, dt_;")
    w("    int   n_;")
    w("    float x_expect_, y_expect_;")
    w("    float x_cont_, y_cont_;")
    w("    float yaw_cont_expect_;")
    w("    float tol_;")
    w("};")
    w("static const OdoGoldenCase k_odo_golden[] = {")
    for name, vx, vy, wz, dt, n in ONE:
        dx, dy, yc = oracle_discrete(vx, vy, wz, dt, n)
        tx, ty, _ = oracle_continuous(vx, vy, wz, dt, n)
        f32 = loop_semi_implicit(vx, vy, wz, dt, n, np.float32)
        tol = tol_for(dx, dy, yc, n, f32)
        w('    {"%s", %s, %s, %s, %s, %d, %s, %s, %s, %s, %s, %s},' % (
            name, cf(vx), cf(vy), cf(wz), cf(dt), n,
            cf(dx), cf(dy), cf(tx), cf(ty), cf(yc), cf(tol)))
    w("};")
    w("static const int k_odo_golden_count = %d;" % len(ONE))
    w("")
    w("// ---- 多段场景：用于「世界系旋转」锚点（先纯转、再走；纯转段位移恒为 0，故精确）----")
    w("struct OdoSegCase {")
    w("    const char* name_;")
    w("    OdoSeg seg_[3];")
    w("    int   seg_count_;")
    w("    float x_expect_, y_expect_, yaw_expect_, tol_;")
    w("};")
    w("static const OdoSegCase k_odo_seg[] = {")
    for name, segs in SEG:
        dx, dy, yc = compose(segs, continuous=False)
        fx, fy, fth = loop_segments_f32(segs)          # 跨段连续，镜像 C++ 的累加
        tol = tol_for(dx, dy, yc, sum(s[4] for s in segs), (fx, fy, fth))
        parts = ", ".join("{%s, %s, %s, %s, %d}" % (cf(a), cf(b), cf(c), cf(d), e)
                          for a, b, c, d, e in segs)
        parts += ", {0.0f, 0.0f, 0.0f, 0.0f, 0}" * (3 - len(segs))
        w('    {"%s", {%s}, %d, %s, %s, %s, %s},' % (
            name, parts, len(segs), cf(dx), cf(dy), cf(yc), cf(tol)))
    w("};")
    w("static const int k_odo_seg_count = %d;" % len(SEG))
    w("")
    w("// ---- 收敛性：相同 T、不同 dt。gap = |离散 \u2212 连续|（一阶格式应随 dt 线性缩小）----")
    w("struct OdoConvCase {")
    w("    const char* name_;")
    w("    float vx_, vy_, wz_, dt_;")
    w("    int   n_;")
    w("    float x_expect_, y_expect_;   // 离散精确解")
    w("    float x_cont_, y_cont_;       // 连续真解")
    w("    float gap_;                   // |离散 \u2212 连续|，期望的一阶误差")
    w("    float tol_;")
    w("};")
    w("static const OdoConvCase k_odo_conv[] = {")
    for name, vx, vy, wz, dt, n in CONV:
        dx, dy, yc = oracle_discrete(vx, vy, wz, dt, n)
        tx, ty, _ = oracle_continuous(vx, vy, wz, dt, n)
        f32 = loop_semi_implicit(vx, vy, wz, dt, n, np.float32)
        tol = tol_for(dx, dy, yc, n, f32)
        w('    {"%s", %s, %s, %s, %s, %d, %s, %s, %s, %s, %s, %s},' % (
            name, cf(vx), cf(vy), cf(wz), cf(dt), n, cf(dx), cf(dy),
            cf(tx), cf(ty), cf(math.hypot(dx - tx, dy - ty)), cf(tol)))
    w("};")
    w("static const int k_odo_conv_count = %d;" % len(CONV))
    w("")
    w("// ---- twist_scale 不变性：scale=k 作用于输入 t  \u2261  scale=1 作用于输入 k\u00b7t ----")
    w("//   \u26a0 设计 \u00a78 锚点 9 原写「位移 \u00d72」只对 \u03c9=0 成立；\u03c9\u22600 时位移不是线性缩放（见 R8）")
    w("struct OdoScaleCase { float scale_, vx_, vy_, wz_; };")
    w("static const OdoScaleCase k_odo_scale[] = {")
    for k, vx, vy, wz in SCALE:
        w("    {%s, %s, %s, %s}," % (cf(k), cf(vx), cf(vy), cf(wz)))
    w("};")
    w("static const int k_odo_scale_count = %d;" % len(SCALE))
    w("")
    w("// ---- wrap 表：借 numpy.angle(exp(i\u03b8)) ----")
    w("struct OdoWrapCase { float yaw_cont_, yaw_wrap_; };")
    w("static const OdoWrapCase k_odo_wrap[] = {")
    for t in WRAP_IN:
        w("    {%s, %s}," % (cf(t), cf(oracle_wrap(t))))
    w("};")
    w("static const int k_odo_wrap_count = %d;" % len(WRAP_IN))
    w("")
    w("// ---- 单步解析值：直行 vx=0.5, dt=0.01, \u03b80=0 \u2192 每步 \u0394x = 0.005（无累积误差）----")
    w("static const float k_odo_step_vx = 0.5f;")
    w("static const float k_odo_step_dt = 0.01f;")
    w("static const float k_odo_step_dx = 0.005f;")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    selfcheck()
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.normpath(os.path.join(here, "..", "odometry_golden.hpp"))
    with open(out, "w") as f:
        f.write(emit())
    print(f"已写入 {out}")
