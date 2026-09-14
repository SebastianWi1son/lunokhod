#!/usr/bin/env python3
"""
plot_odometry.py — 把 odometry_demo 导出的 CSV 画成四格图

用法：
    python3 tools/plot_odometry.py run.csv run.png

四格：
    ① 轨迹 (x, y)         —— 实走路线
    ② yaw 出口            —— 验证 wrap 行为
    ③ 打滑信号            —— cmd 期望 vs twist 实际
    ④ 半隐式 vs 显式       —— 终点放大，真解居中（与 odometry_design §7b R14 对应）

为什么要这一格 ④：显式欧拉与半隐式欧拉的误差【幅值完全相同、方向相反】
（真解/显式 = e^{+iφ/2}·sinc(φ/2)，真解/半隐式 = e^{−iφ/2}·sinc(φ/2)），
放大后可以看见真解正好夹在两者中间。这不是“谁更准”，是“一个超前一个滞后”。
"""

import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402


def use_cjk_font():
    """有中文字体就用，没有就退回英文标签（避免画出一堆方块）。"""
    for name in ["Noto Sans CJK JP", "Noto Sans CJK SC", "Source Han Sans SC",
                 "WenQuanYi Zen Hei", "Microsoft YaHei", "SimHei"]:
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


def load(path):
    rows = list(csv.DictReader(open(path)))
    g = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    return {
        "t":   np.array([float(r["tick"]) * float(r["dt"]) for r in rows]),
        "x":   g("x"), "y": g("y"), "yaw": g("yaw"),
        "cvx": g("cmd_vx"), "cvy": g("cmd_vy"),
        "tvx": g("twist_vx"), "tvy": g("twist_vy"),
    }


def integrate(kind, vx, wz, dt, n):
    """本地重算：kind = 'semi'（先 yaw 后位移）或 'expl'（先位移后 yaw）"""
    xs, ys, th = [0.0], [0.0], 0.0
    for _ in range(n):
        if kind == "semi":
            th += wz * dt
        c, s = math.cos(th), math.sin(th)
        xs.append(xs[-1] + vx * c * dt)
        ys.append(ys[-1] + vx * s * dt)
        if kind == "expl":
            th += wz * dt
    return np.array(xs), np.array(ys)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "run.csv"
    dst = sys.argv[2] if len(sys.argv) > 2 else "run.png"
    zh = use_cjk_font()
    L = (lambda cn, en: cn if zh else en)  # noqa: E731

    d = load(src)
    fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))

    # ① 轨迹
    ax[0, 0].plot(d["x"], d["y"], lw=1.8, color="#1f77b4")
    ax[0, 0].plot(d["x"][0], d["y"][0], "go", ms=10, label=L("起点", "start"))
    ax[0, 0].plot(d["x"][-1], d["y"][-1], "rs", ms=10, label=L("终点", "end"))
    ax[0, 0].set_aspect("equal")
    ax[0, 0].grid(alpha=0.3)
    ax[0, 0].legend(fontsize=9)
    ax[0, 0].set_title(L("① 轨迹 (x, y) —— 由 odometry 积分得到", "① trajectory"), fontsize=11)
    ax[0, 0].set_xlabel("x (m)")
    ax[0, 0].set_ylabel("y (m)")

    # ② yaw 出口
    ax[0, 1].plot(d["t"], d["yaw"], lw=1.8, color="#d62728")
    ax[0, 1].axhline(math.pi, ls="--", c="gray", lw=1)
    ax[0, 1].axhline(-math.pi, ls="--", c="gray", lw=1)
    ax[0, 1].grid(alpha=0.3)
    ax[0, 1].set_title(L("② yaw 出口（wrap 到 (-π, π]）", "② yaw output (wrapped)"), fontsize=11)
    ax[0, 1].set_xlabel("t (s)")
    ax[0, 1].set_ylabel("yaw (rad)")

    # ③ 打滑信号
    ax[1, 0].plot(d["t"], d["cvy"], lw=1.8, label=L("cmd.vy（期望）", "cmd.vy (wanted)"))
    ax[1, 0].plot(d["t"], d["tvy"], lw=1.8, label=L("twist.vy（实际）", "twist.vy (actual)"))
    ax[1, 0].fill_between(d["t"], d["tvy"], d["cvy"], where=(d["cvy"] > d["tvy"]),
                          color="red", alpha=0.25,
                          label=L("差值 = L0 打滑信号", "gap = L0 slip signal"))
    ax[1, 0].grid(alpha=0.3)
    ax[1, 0].legend(fontsize=9)
    ax[1, 0].set_title(L("③ 打滑信号：期望 vs 实际", "③ slip signal: cmd vs twist"), fontsize=11)
    ax[1, 0].set_xlabel("t (s)")
    ax[1, 0].set_ylabel("vy (m/s)")

    # ④ 半隐式 vs 显式 vs 真解
    vx, wz, h, n = 0.3, 0.7, 0.01, 200
    sx, sy = integrate("semi", vx, wz, h, n)
    ex, ey = integrate("expl", vx, wz, h, n)
    T = wz * h * n
    tx, ty = (vx / wz) * math.sin(T), (vx / wz) * (1 - math.cos(T))
    ax[1, 1].plot(sx, sy, lw=2.2, color="#1f77b4", label=L("半隐式（odometry）", "semi-implicit"))
    ax[1, 1].plot(ex, ey, lw=2.2, ls="--", color="#ff7f0e", label=L("显式（demo 原写法）", "explicit"))
    ax[1, 1].plot([sx[-1]], [sy[-1]], "o", color="#1f77b4", ms=8)
    ax[1, 1].plot([ex[-1]], [ey[-1]], "s", color="#ff7f0e", ms=8)
    ax[1, 1].plot([tx], [ty], "k*", ms=20, label=L("真解（SE(2) 闭式）", "truth (SE(2) closed form)"))
    ax[1, 1].grid(alpha=0.3)
    ax[1, 1].legend(fontsize=9, loc="upper left")
    half = 1.6e-3
    ax[1, 1].set_xlim(tx - half, tx + half)
    ax[1, 1].set_ylim(ty - 1.9e-3, ty + 1.4e-3)
    ax[1, 1].set_aspect("equal")
    ax[1, 1].set_title(L("④ 半隐式 vs 显式：终点放大 ~600×（真解居中）",
                         "④ semi vs explicit, endpoint zoomed"), fontsize=11)
    ax[1, 1].set_xlabel("x (m)")
    ax[1, 1].set_ylabel("y (m)")

    plt.tight_layout()
    plt.savefig(dst, dpi=110)
    print(f"已保存 {dst}")
    print(f"  半隐式终点 ({sx[-1]:.9f}, {sy[-1]:.9f})")
    print(f"  显式终点   ({ex[-1]:.9f}, {ey[-1]:.9f})")
    print(f"  真解       ({tx:.9f}, {ty:.9f})")


if __name__ == "__main__":
    main()
