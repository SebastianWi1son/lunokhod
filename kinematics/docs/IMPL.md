---
class: status
generated: false
---
> **类：C 状态** —— **跟代码变**。⚠️ 与代码天然重复，建议降级为「一屏文件地图 + 追加式变更记录」，见 trash/README.md。
> 文档体系与写作规则：../../docs/README.md

# Kinematics 实现真相文档（以代码为准）

> 生成日期：2026-08-22 ｜ 最后同步：2026-09-14（odometry 落地：140 断言全绿 + ctest 注册 + reset(Pose)）
> 本文档描述 `kinematics/` 目录下**实际存在的代码**，一切以源码为准。
> 同目录（`kinematics/docs/`）的 DESIGN.md / DEV_GUIDE.md / THEORY.md 是设计文档，`log/` 下是阶段复盘，与现状的差异见 §7，问题见 §8。
> （2026-09-13 文档卫生整理：这些文档从根 `docs/` 归位到本目录。）
> 维护规则：**改代码必须同步改本文档**；本文档与代码冲突时，以代码为准并当场修正文档。

---

## 1. 项目定位

C++17 header-only 零依赖嵌入式底盘运动学库：输入本体系 `Twist (vx, vy, wz)`，输出各轮角速度 `WheelSpeeds`，支持差速 / Mecanum / 全向三类线性底盘。编译期模板（CRTP）派发，无虚函数、零堆、零异常，仅依赖 `<cstdint>` / `<cmath>`。

注意：设计文档中规划的 `speed_limiter.hpp`（STAGE 3）**不在本模块内**——它已演化为独立模块 `control/twist_acc_limiter/`（STATIC 库，见该模块 docs）。

## 2. 实际目录结构

```
kinematics/
├── inc/                        ← 头文件根（设计文档写的是 include/，实际是 inc/）
│   ├── chassis.hpp             ← **瞬时映射**对外聚合入口（用户只 include 这一个）
│   ├── kinematics.hpp          ← 数学核心：Kinematics<Derived> CRTP 基类 + jacobian_apply + k2PI
│   ├── drive_diff.hpp          ← DiffDrive
│   ├── drive_mecanum.hpp       ← MecanumDrive
│   └── drive_omni.hpp          ← OmniDrive
├── test/
│   └── test_kinematics.cpp     ← 三底盘测试（手写 CHECK 宏，退出码即结果）
├── examples/
│   ├── example.cpp             ← 三底盘 API 调用演示（CMake target: example_diff）
│   └── simulation_demo.cpp     ← 2D 位姿积分仿真（CMake target: simulation_demo）
├── legacy/                     ← 2024 年 C 语言 nav 参考实现（不属于本库）
├── CMakeLists.txt
└── AGENTS.md                    ← 代理协作规则（inc/ 人写、test/ AI 写）
```

> **2026-09-15 迁出说明**：原先在本目录下的 `inc/contracts.hpp`、`inc/odometry.hpp`
> 及其测试 / 金标生成器 / 工具已搬到两个新组件：
>
> | 原位置 | 现位置 |
> |---|---|
> | `inc/contracts.hpp` | [`../../contracts/inc/contracts.hpp`](../../contracts/inc/contracts.hpp) |
> | `inc/odometry.hpp` · `test/test_odometry.cpp` · `test/odometry_golden.hpp` · `test/tools/gen_odometry_golden.py` · `tools/odometry_demo.cpp` · `tools/plot_odometry.py` · `docs/ODOMETRY_DESIGN.md` · `docs/log/ODOMETRY_FAQ.md` | [`../../odometry/`](../../odometry/) （见其 [`IMPL.md`](../../odometry/docs/IMPL.md)） |
>
> 因此本目录的 `tools/` 与 `test/tools/` 已不存在。

## 3. 数据契约（现在位于 `../../contracts/inc/contracts.hpp`）

> 契约提为独立库（2026-09-15）：它们不只属 kinematics，odometry 与 twist_acc_limiter 也靠它。
> 下面是这些类型的定义 —— 它们是**本库的对外接口面**，所以仍在本代码地图里讲。

```cpp
struct Twist {          // 输入：本体系速度
    float vx_;          // 前进 m/s
    float vy_;          // 侧移 m/s（差速底盘忽略）
    float wz_;          // 角速度 rad/s
};
struct WheelSpeeds {    // 输出：各轮角速度 rad/s
    uint8_t count_;     // 有效轮数
    float values_[6];   // 定长上限 6，前 count_ 个有效
};
struct Pose {           // 里程计输出：世界系位姿（O3 定案：跨组件输出契约归数据层）
    float x_, y_, yaw_; // m / rad（yaw_ 是 wrap 值；连续值走 yaw_continuous()）
};
```

字段名带尾下划线（`vx_`）。注意：POD 数据契约的成员用尾下划线是私有成员的惯例，与"对外契约"定位略有风格张力（见 §8-P5，仅记录不评判）。

## 4. 数学核心（inc/kinematics.hpp）

- `constexpr float k2PI = 6.283185307179586f;`
- `Kinematics<Derived>`：CRTP 能力基类（"底盘具备运动学能力"，Comparable 模式）。接口层方法 `inverse_kinematics` / `forward_kinematics` 均 `const`，内部 `static_cast<const Derived*>(this)->xxx_impl()` 派发到实现层。
- `jacobian_apply`（翻译官，矩阵 × 向量）：

```cpp
template<uint8_t N>
WheelSpeeds jacobian_apply(const float (&J)[N][3], uint8_t wn, const Twist& t_cmd) {
    WheelSpeeds out_ws;
    out_ws.count_ = wn;
    for (uint8_t i = 0; i < wn; ++i) {         // 循环到 wn（运行时轮数），N 只是数组容量
        out_ws.values_[i] = J[i][0]*t_cmd.vx_ + J[i][1]*t_cmd.vy_ + J[i][2]*t_cmd.wz_;
    }
    return out_ws;
}
```

模板参数 `N` 由数组声明尺寸推导（**数组容量上界**）；`wn` 是运行时有效行数。两者只在 OmniDrive（声明 `J[6][3]`、实际 `wn_` 行）处不相等——循环上界必须用 `wn`，否则读写未初始化的 `J[3..5]` 行。（2026-09-13 已修复，见 §8-P1）

## 5. 三种底盘（以代码为准的数学）

### 5.1 DiffDrive（inc/drive_diff.hpp）

- 构造：`DiffDrive(float wheelbase, float wheel_radius)`，成员 `wb_`（轮距 m）、`r_`（轮半径 m）。
- 逆运动学（等价公式，代码直接把 1/r 折进 J）：

```
J = [ 1/r   0   -wb/(2r) ]     values_[0] = 左轮
    [ 1/r   0   +wb/(2r) ]     values_[1] = 右轮
ωL = (Vx − wz·wb/2)/r    ωR = (Vx + wz·wb/2)/r
```

- 正运动学：`vx = r(ωR+ωL)/2`，`wz = r(ωR−ωL)/wb`，`vy ≡ 0`（非完整约束，静默置零不报错）。

### 5.2 MecanumDrive（inc/drive_mecanum.hpp）

- 构造：`MecanumDrive(float lx, float ly, float r)`（半轮距 lx、半轴距 ly、轮半径 r）。
- 轮序约定（与 J 行序一致）：`values_[0]=FL, [1]=FR, [2]=RL, [3]=RR`。

```
J = [ 1  -1  -(lx+ly) ]      每行再整体除以 r
    [ 1   1  +(lx+ly) ]
    [ 1   1  -(lx+ly) ]
    [ 1  -1  +(lx+ly) ]
```

- 正运动学（4 轮伪逆的展开形式）：

```
vx =  r(FL+FR+RL+RR)/4
vy =  r(−FL+FR+RL−RR)/4
wz =  r(−FL+FR−RL+RR)/(4(lx+ly))
```

### 5.3 OmniDrive（inc/drive_omni.hpp）—— ⚠ 符号约定与 DESIGN.md 不同

- 构造：`OmniDrive(uint8_t wn, float cr, float gamma, float wr)`（轮数、轮心距 R、首轮偏移角 γ、轮半径 r）。
- 逆运动学（**代码实际采用**，θᵢ = i·2π/wn + γ）：

```
ωi = ( −sin(θᵢ)·Vx + cos(θᵢ)·Vy + R·wz ) / r
```

DESIGN.md 写的是 `+sin(θᵢ)·Vx − cos(θᵢ)·Vy − R·wz`（三处符号全部相反）。两者互为镜像约定，测试锚点（直行 → `{0, −14.434, +14.434}` 等）已按代码约定验证互逆自洽。**接线/对轮时以本文档的代码约定为准。**
- 正运动学：**通用 N 轮伪逆**（2026-09-13 由 3 轮 γ=0 特例升级；对任意 `wn_`、任意 `gamma_` 成立）：

```
vx = (2/N)·Σ(−sin θᵢ)·uᵢ       θᵢ = i·2π/N + γ,  uᵢ = r·ωᵢ,  N = wn_
vy = (2/N)·Σ( cos θᵢ)·uᵢ
wz = Σuᵢ / (N·R)
```

`wn_=3, gamma_=0` 时可化简回旧特例公式（`vx=(u2−u1)/√3`、`vy=(2u0−u1−u2)/3`、`wz=(u0+u1+u2)/(3R)`），向后兼容。互逆性已由锚点覆盖：n=3 γ=0、n=4、n=3 γ=30°。

## 6. 构建与测试（实测）

- CMake ≥ 3.28，C++17。`add_library(kinematics INTERFACE)`（header-only），`target_include_directories` 暴露 `inc/`，`INTERFACE` 依赖 `contracts`。
- **三个** target，**全部**启用 `-Wall -Wextra -Werror`：`test_kinematics`、`example_diff`（= examples/example.cpp）、`simulation_demo`（L3 2D 位姿积分 demo）。
  （原先还有 `test_odometry`，2026-09-15 随 odometry 组件迁出。）
- 2026-09-14 补了 `enable_testing()` + `add_test`：`ctest --test-dir build --output-on-failure` 现在**真能跑**（此前报 “No tests were found”）。
- 测试方法论：锚点（手算有理数如 `50.0f/3.0f`）+ 互逆（forward∘inverse ≈ 恒等）+ 边界（零输入）；容差 1e-5，退出码即结果。
- 手动编译命令（AGENTS.md 记载）：`g++ -std=c++17 -Wall -Wextra -Iinc test/test_kinematics.cpp`
- **2026-08-22 实测**：`g++ -std=c++17 -Wall -Wextra -Werror -Iinc test/test_kinematics.cpp` → `ALL PASS`，退出码 0。
- **2026-09-14 实测**：`ctest` → **4/4 Passed**；`test_odometry` → **140/140 断言**，零告警。
- **2026-09-15 实测（odometry 迁出后）**：聚合 `ctest` → **8/8 Passed**；单库 `cmake -S kinematics` → **3/3 Passed**。

## 7. 与根目录设计文档的差异（漂移清单）

| # | 设计文档说 | 代码现实 |
|---|---|---|
| 1 | 目录 `include/kinematics/`、`tests/` | 实际 `inc/`、`test/` |
| 2 | 文件名 `differential_drive.hpp` 等 | 实际 `drive_diff.hpp` / `drive_mecanum.hpp` / `drive_omni.hpp` |
| 3 | 基类名 `Chassis<Derived>`，`chassis.hpp` 是基座 | 实际基类叫 `Kinematics<Derived>`（STAGE2 复盘已修正命名，DESIGN.md 未回改）；`chassis.hpp` 是聚合入口 |
| 4 | STAGE 3 在 kinematics 内做 header-only `speed_limiter.hpp` | 实际落地为 `control/twist_acc_limiter/`（.hpp+.cpp 的 STATIC 库），且不进 chassis.hpp |
| 5 | Omni 逆运动学公式 sin/−cos/−R | 代码为 −sin/+cos/+R（§5.3） |
| 6 | Omni forward 计划"3 轮特例先行，N>3 用一般公式" | 已实现通用 N 轮伪逆（含 γ）；3 轮 γ=0 是其中 N=3 的化简 |
| 7 | 统一接口 `inverse<Chassis>(cmd)` 自由函数 | 实际为成员调用 `chassis.inverse_kinematics(cmd)`（DEV_GUIDE 备忘 4 已承认此修正） |
| 8 | 开发计划 STAGE 3"待开发" | 其功能已由 twist_acc_limiter 模块完成 |

## 8. 问题清单

- **P1（正确性隐患）✅ 已修复 2026-09-13**：`inc/kinematics.hpp` `jacobian_apply` 循环上界用编译期 `N` 而非运行时 `wn`；OmniDrive 传 `J[6][3]` + `wn=3` 时读取未初始化的 `J[3..5]`（UB，测试碰巧全绿）。→ 已改为 `i < wn`，并补 n=4 / γ≠0 互逆锚点。
- **P2（功能缺口）✅ 已修复 2026-09-13**：`drive_omni.hpp` `forward_impl` 硬编码 3 轮 γ=0，`wn_`/`gamma_` 未使用，`wn>3` 输出错误且无断言。→ 已改为通用 N 轮伪逆（含 γ）。
- **P3（构建缺口）✅ 已修复 2026-09-13**：`examples/simulation_demo.cpp` 没有 CMake target。→ 已挂 `simulation_demo` target（`-Werror`，编译通过、运行退出码 0）。
- **P4（不一致）✅ 已修复 2026-09-13**：`example_diff` target 未加 `-Wall -Wextra -Werror`。→ 已补齐。
- **（新增）P7（未开工）** OmniDrive 构造无参数校验：`J[6][3]` 定长，`wn>6` 越界写、`wn<2` 数学无意义——待加断言/校验（对应根 `../../docs/TODO.md` P12）。
- **P5（风格）** `contracts.hpp` 的 POD 契约成员用尾下划线命名（`vx_`），与"公共契约"语义存在张力；STAGE1_REVIEW 的 struct-POD 讨论未涉及此点。
- **P6（文档债）** `DESIGN.md` 与 `DEV_GUIDE.md`（2026-09-13 已从根 `docs/` 归位到本目录）未随 STAGE2 命名修正与 STAGE3 移址回改（§7 全部条目）。

---

## 9. 增量记录

> **2026-09-15：odometry 已独立成组件，本节整段迁走。**
>
> 迁移去向（**内容未删减，只是换了家**）：
>
> | 原本节内容 | 现位置 |
> |---|---|
> | §9.1 接口 / §9.2 参数分两类 / §9.3 行为要点 / §9.7 已知未闭合 | [`../../odometry/docs/IMPL.md`](../../odometry/docs/IMPL.md) |
> | §9.4 测试与金标 / §9.6 开发-分析工具 | 同上 |
> | §9.5 验收记录（含 3 个坑） | 同上，规则进 [`../../odometry/AGENTS.md`](../../odometry/AGENTS.md) §4 错误账本 |
>
> **为什么迁走**：odometry 的依赖只有 `contracts`（与运动学正/逆解无关），
> 它是有状态、可选、非所有用户都需要的组件 —— 已提为独立库 `../../odometry/`。
> 本节留在本文件里，等于 kinematics 的代码地图在记别的库。
>
> 本组件自己的增量记录：**暂无**（下次改本库存代码时从这里往下追加）。
