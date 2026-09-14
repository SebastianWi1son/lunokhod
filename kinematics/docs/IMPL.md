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
│   ├── contracts.hpp           ← 数据契约：Twist / WheelSpeeds / Pose（POD）
│   ├── kinematics.hpp          ← 数学核心：Kinematics<Derived> CRTP 基类 + jacobian_apply + k2PI
│   ├── drive_diff.hpp          ← DiffDrive
│   ├── drive_mecanum.hpp       ← MecanumDrive
│   ├── drive_omni.hpp          ← OmniDrive
│   └── odometry.hpp            ← 里程计：有状态积分器 + 可选记录（**opt-in，不进 chassis.hpp**）
├── test/
│   ├── test_kinematics.cpp     ← 三底盘测试（手写 CHECK 宏，退出码即结果）
│   ├── test_odometry.cpp       ← 里程计测试（140 断言，oracle 全部外部）
│   ├── odometry_golden.hpp     ← ⚠ 自动生成，勿手改（gen_odometry_golden.py 产物）
│   └── tools/gen_odometry_golden.py  ← 金标生成器（scipy / sympy / numpy 交叉验证）
├── examples/
│   ├── example.cpp             ← 三底盘 API 调用演示（CMake target: example_diff）
│   └── simulation_demo.cpp     ← 2D 位姿积分仿真（CMake target: simulation_demo）
├── src/                        ← 空目录（待清理：删掉，或说明为何保留）
├── legacy/                     ← 2024 年 C 语言 nav 参考实现（不属于本库）
├── CMakeLists.txt
└── AGENTS.md                    ← 代理协作规则（inc/ 人写、test/ AI 写）
```

## 3. 数据契约（inc/contracts.hpp）

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

- CMake ≥ 3.28，C++17。`add_library(kinematics INTERFACE)`（header-only），`target_include_directories` 暴露 `inc/`。
- **四个** target，**全部**启用 `-Wall -Wextra -Werror`：`test_kinematics`、`example_diff`（= examples/example.cpp）、`simulation_demo`（L3 2D 位姿积分 demo）、`test_odometry`（**条件注册**：`inc/odometry.hpp` 存在才挂上）。
- 2026-09-14 补了 `enable_testing()` + `add_test`：`ctest --test-dir build --output-on-failure` 现在**真能跑**（此前报 “No tests were found”）。
- 测试方法论：锚点（手算有理数如 `50.0f/3.0f`）+ 互逆（forward∘inverse ≈ 恒等）+ 边界（零输入）；容差 1e-5，退出码即结果。
- 手动编译命令（AGENTS.md 记载）：`g++ -std=c++17 -Wall -Wextra -Iinc test/test_kinematics.cpp`
- **2026-08-22 实测**：`g++ -std=c++17 -Wall -Wextra -Werror -Iinc test/test_kinematics.cpp` → `ALL PASS`，退出码 0。
- **2026-09-14 实测**：`ctest` → **4/4 Passed**；`test_odometry` → **140/140 断言**，零告警。

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

## 9. 增量记录：Odometry（2026-09-14）

> 本节按「**增量记录**」格式追加，不改写前面章节。
> （IMPL 的目标形态：**一屏文件地图 + 追加式变更记录**，不再逐行复述代码）

**文件**：`inc/odometry.hpp`（**opt-in，不进 `chassis.hpp`**）；`contracts.hpp` 增加 `Pose`。

**性质**：本库**唯一有状态**的组件（与 `PID` 同类；`kinematics` 全是纯函数）。
职责 = **积分 + 可选记录**（`sink == nullptr` 时一帧不记）。

### 9.1 接口（5 个公开成员）

| 成员 | 语义 |
|---|---|
| `Odometry(float twist_scale = 1.0f)` | 唯一构造参数 = 标定系数（**硬件属性**） |
| `set_sink(SampleSink, void* ctx)` | 注册数据出口；nullptr = 不记录 |
| `reset()` / `reset(const Pose&)` | **只清位姿**；`twist_scale_` / `sink_` **保留**（清状态，不清配置）。R12：带参重载 = 设到指定位姿 |
| `update(twist, dt, tick, cmd, ws) → Pose` | 每帧一次；返回值与 `pose()` 同源 |
| `pose()` / `yaw_continuous()` / `yaw_ref()` | wrap / 连续 / 连续（融合层校准源） |

### 9.2 参数分两类（本组件最容易读错的地方）

| 类别 | 字段 |
|---|---|
| **参与计算** | `body_twist`、`dt` |
| **只作记录** | `tick`、`cmd`、`ws` |

### 9.3 行为要点

- **积分**：半隐式欧拉（R6）—— 先 `yaw_ += wz·dt`，再用**新** yaw 的 `cos/sin` 旋转位移。
- **标定**：`twist_scale_` 乘 **vx / vy / wz 三个通道**（漏 `wz` → 每圈丢约 11°）。
- **记录**：`ws == nullptr` 时**整帧不记**（R7）；样本里 `twist_` 是**标定后**的值，`cmd_` **原样**。
- **yaw**：内部只存**一个连续值**；两个出口各取所需（连续 vs wrap），**不存第二个成员**。

### 9.4 测试与金标（oracle 全部外部）

| 文件 | 作用 |
|---|---|
| `test/test_odometry.cpp` | **140 断言，11 组**（金标 / 世界系旋转 / 收敛 / 零输入冻结 / reset / wrap / 单步 / scale 不变性 / sink / R7 防御 / **初始位姿（R12）**） |
| `test/tools/gen_odometry_golden.py` | 金标生成器（**五重自检**，打印闭式 vs 循环 vs scipy 的偏差） |
| `test/odometry_golden.hpp` | ⚠ 自动生成，勿手改 |

**oracle 来源**：等比级数闭式（离散精确解）+ SE(2) 指数映射（连续真解，scipy DOP853 交叉验证）+ `numpy.angle`（wrap）+ **实测推导的容差**。

### 9.5 验收记录

- 手写者：作者本人（AI 未改 `inc/`）；测试由 AI 编写。
- 首次手敲出现 3 个坑（契约字段名 / `vy*sin` / 同类型字段静默错位），**全部被测试抓到**；规则已入 `../AGENTS.md` 错误账本。
- `ctest` → **4/4 Passed**；`test_odometry` → **140/140**；`-Wall -Wextra -Werror` 零告警。

### 9.6 开发 / 分析工具

| 文件 | 作用 |
|---|---|
| `tools/odometry_demo.cpp` | 四段场景（直行 / 圆弧 / 麦轮横移（注入 40% 打滑）/ 原地转）→ 导 `run.csv`（850 帧 × 15 列） |
| `tools/plot_odometry.py` | `run.csv` → 四格图（轨迹 / yaw 出口 / 打滑信号 / 半隐式 vs 显式放大对比） |

```bash
cmake -S kinematics -B build && cmake --build build -j
./build/odometry_demo run.csv
python3 kinematics/tools/plot_odometry.py run.csv run.png
```

> 这两个是**工具**，不是库示例 —— 它们依赖 `inc/odometry.hpp`，但库本体不依赖它们。
> **用途**：CSV 是打滑检测 / 底盘建模 / 复盘分析的原料（见 `../../docs/TODO.md` P19），
> 也是将来实车数据回放与 PC 侧算法验证的入口。

### 9.7 已知未闭合

见 [`ODOMETRY_DESIGN.md`](ODOMETRY_DESIGN.md) §7b：
**R12**（无初始位姿入口 —— 待拍板是否加 `reset(const Pose&)`）、
**R13**（记录未解耦 —— 等第二个需要记录的组件出现再重开讨论）。
