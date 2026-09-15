---
class: status
generated: false
---
> **类：C 状态** —— **跟代码变**。⚠️ 与代码天然重复，建议降级为「一屏文件地图 + 追加式变更记录」，见 trash/README.md。
> 文档体系与写作规则：../../../docs/README.md

# TwistAccLimiter 实现真相文档（以代码为准）

> 生成日期：2026-08-22
> 本文档描述 `control/twist_acc_limiter/` 下**实际存在的代码**，一切以源码为准。
> 根目录 `docs/` 的 DEV_GUIDE.md STAGE 3 是本模块的设计源头（当时叫 `speed_limiter.hpp`，规划放 kinematics 内 header-only），与现状的差异见 §6，问题见 §7。
> 维护规则：**改代码必须同步改本文档**；冲突时以代码为准并当场修正文档。

---

## 1. 项目定位

Twist 空间三通道独立加速度限幅器（斜坡发生器 / slew rate limiter）：上游指令（导航/PID/遥控，可任意跳变）→ 每通道每帧最多变化 `acc·dt` 的平滑输出 → 下游逆运动学。

- 在 ARCHITECTURE.md 下行链中占位 ①：`SpeedLimiter (Vx,Vy,ω) → 限幅后 (Vx,Vy,ω)`，是"**先限幅、后分解**"原则的执行者（限幅约束车体三量，不在轮速空间限，保运动学一致性）。
- 本模块是 kinematics STAGE 3 的实际落地形态，但**独立成模块**（`control/twist_acc_limiter/`，STATIC 库），不进 kinematics 的 `chassis.hpp` 聚合入口。
- 契约复用：`Twist` 类型 include 自 **`contracts`** 库的 `contracts.hpp`（**单一事实来源**）。
  （2026-09-15：原先写的是“include 自 kinematics 的 contracts.hpp”—— 
  契约已提为最底层库 `contracts/`，本组件对 kinematics 的依赖已断开。）
- v1 范围（与 DESIGN.md 一致）：仅每通道加速度斜坡。**不含**速度上限、Jerk、联合约束（属后续/应用层职责）。

## 2. 实际目录结构

```
control/twist_acc_limiter/
├── inc/twist_acc_limiter.hpp        ← 公开 API：LimitResult + TwistAccLimiter（声明）
├── src/twist_acc_limiter.cpp        ← 实现（声明/定义分离 → STATIC 库）
├── test/test_twist_acc_limiter.cpp  ← 锚点测试 T1~T6 + 防御回归
├── examples/example_twist_acc_limiter.cpp  ← 链式演示：限幅 → DiffDrive 逆解 → 轮速
├── docs/                            ← 本文档
├── CMakeLists.txt
└── build/ , cmake-build-debug/      ← IDE/构建产物（非源码）
```

## 3. 公开 API（inc/twist_acc_limiter.hpp，逐字为准）

```cpp
struct LimitResult {
    Twist out_;                              // 限幅后的输出
    bool is_vx_lim_, is_vy_lim_, is_wz_lim_; // 各通道本帧是否被限幅（撞墙标志）
};

class TwistAccLimiter {
public:
    TwistAccLimiter(float acc_vx, float acc_vy, float acc_wz);  // 单位: m/s², m/s², rad/s²
    LimitResult limit(const Twist& t_cmd, float dt);            // 每帧调用，dt 注入
    void reset();                                               // 状态清零（重新从静止起步）
private:
    static float ramp(float cmd, float prev, float acc, float dt, bool& is_lim);
    Twist acc_;    // 三通道加速度上限（唯一配置）
    Twist prev_;   // 上一帧输出（唯一运行状态，构造/reset 后为 {0,0,0}）
};
```

## 4. 行为语义（src/twist_acc_limiter.cpp，以代码为准）

### 4.1 正常路径（dt > 0）

每通道独立执行 `ramp()`：把目标夹到 `[prev − acc·dt, prev + acc·dt]` 区间内。

```
max_step = acc · dt
lo = prev − max_step,  hi = prev + max_step
is_lim = (cmd < lo) || (cmd > hi)      // 区间含端点：恰好到达 == 未限幅
out  = clamp(cmd, lo, hi)
```

帧末 `prev_ = out`（状态更新）。三通道完全独立——vx 撞墙不影响 vy/wz 直通。

### 4.2 防御路径（dt ≤ 0）—— 与设计伪代码有一处实质差异

实际代码：

```cpp
if (dt <= 0.0f) {
    prev_ = t_cmd;                          // ← 状态同步
    return {t_cmd, false, false, false};    // 直通，不阻挡指令
}
```

直通（安全侧选择，与设计一致）**且同步 `prev_ = t_cmd`**。DEV_GUIDE 3.3 伪代码只直通不同步——那会导致 dt 恢复后输出从旧 prev 回跳。测试文件头注释"回跳回归（L1 修复验证）"表明这是开发中实测修复过的 bug，现已有回归用例锁死：dt 非法帧之后下一帧不回跳。

### 4.3 状态与生命周期

- 构造：`acc_` 注入即定死（无 setter）；`prev_ = {0,0,0}`（从静止起步）。
- `reset()`：`prev_ = {0,0,0}`，等价 legacy `dsp_traj_reset` 语义（停赛/重新起步）。
- 本类是整个 lunokhod 里**唯一有状态**的类（kinematics 三底盘全部是无状态纯函数 const 方法）；`limit()` 因更新 `prev_` 而是**非 const**。

## 5. 构建与测试（实测）

- CMake ≥ 3.28，C++17。`add_subdirectory(../../kinematics)` 取 INTERFACE 库（include 路径自动传播，Twist 契约唯一来源）。
- `add_library(twist_acc_limiter STATIC src/twist_acc_limiter.cpp)`，PUBLIC 暴露 `inc/`、PUBLIC 链接 kinematics；`-Wall -Wextra -Werror`（PRIVATE，三个 target 全配齐，比 kinematics 的 CMake 更一致）。
- 三个 target：`twist_acc_limiter` / `test_twist_acc_limiter` / `example_twist_acc_limiter`。
- 测试内容（T1~T6 + 防御）：起步爬坡、停止下坡、过零反转、未饱和直通、三通道隔离+饱和标志、稳态保持、dt≤0 直通与回跳回归。期望值全部手算有理数（0.1 步进），容差 1e-5，退出码即结果。
- **2026-08-22 实测**：`g++ -std=c++17 -Wall -Wextra -Werror -Iinc -I../../kinematics/inc test/test_twist_acc_limiter.cpp src/twist_acc_limiter.cpp` → `ALL PASS ✅`，退出码 0。
- example 演示完整下行链：`TwistAccLimiter(1,1,2)` → `DiffDrive(0.16,0.03)` 逆解，cmd={0.5,0.3,1.0} 阶跃、dt=10ms，每 10 帧打印限幅输出/饱和标志/左右轮速；终态锚点 ωL=(0.5−0.08)/0.03=14.00、ωR=19.33（手算吻合）。

## 6. 与设计文档的差异（漂移清单）

| # | 设计文档说（DEV_GUIDE STAGE 3 / DESIGN.md / ARCHITECTURE.md） | 代码现实 |
|---|---|---|
| 1 | kinematics 内 `speed_limiter.hpp`（当时名，header-only），STAGE 3“待开发” | 独立模块 `control/twist_acc_limiter/`，.hpp+.cpp 声明定义分离，STATIC 库。**设计文档已于 2026-09-14 回改** |
| 2 | 类名 `SpeedLimiter` | `TwistAccLimiter`（不在 3.6 命名候选表内，最终命名未回写任何决策记录） |
| 3 | `LimitResult { out; vx_lim, vy_lim, wz_lim; }` | `LimitResult { out_; is_vx_lim_, is_vy_lim_, is_wz_lim_; }`（字段名不同） |
| 4 | dt≤0 直通返回、**不更新** prev_ | 直通**且** `prev_ = t_cmd`（修复回跳 bug，含回归测试；伪代码是过时真相） |
| 5 | 结构化绑定用法 `auto [out, vx_lim, ...] = limiter.limit(...)` | 实际代码/测试用 `r.out_`、`r.is_vx_lim_` 成员访问 |
| 6 | DESIGN.md 架构树把 speed_limiter.hpp 画在 kinematics 内 | ✅ **2026-09-14 已回改**（DESIGN.md §架构 现写 `control/twist_acc_limiter/`） |
| 7 | 上报机制"方案 a LimitResult 返回式" | ✅ 一致，已实现（AGENTS.md 记录的选型落地） |

## 7. 问题清单（只记录，不修改）

- **P1（测试缺口）** `reset()` 无任何测试用例（T1~T6 + 防御均未覆盖）；构造后首帧行为有覆盖，reset 后首帧行为没有。
- **P2（文档债）** 最终命名 `TwistAccLimiter`、模块从 kinematics 迁出到 control/ 这两个决策，未回写到根 docs/（DESIGN.md 架构树、DEV_GUIDE STAGE 3、AGENTS.md 的"speed_limiter.hpp"表述全部过时）。
- **P3（文档债）** DEV_GUIDE 3.3 伪代码的 dt≤0 分支与实现相反（§6-4），后续照伪代码复刻会重新引入回跳 bug——建议作者在根文档或本文档锚定"以本文档 §4.2 为准"。
- **P4（覆盖缺口，属已知范围）** 无速度上限（max_vel）、无 Jerk、无联合约束——DESIGN.md 已声明为 v1 刻意不覆盖，非缺陷，列此存档。
- **P5（卫生）** 仓库目录里留有 `build/`、`cmake-build-debug/` 构建产物目录，无 .gitignore 管理（若入 git 会污染版本库）。
