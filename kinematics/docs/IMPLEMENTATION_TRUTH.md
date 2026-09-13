# Kinematics 实现真相文档（以代码为准）

> 生成日期：2026-08-22
> 本文档描述 `kinematics/` 目录下**实际存在的代码**，一切以源码为准。
> 根目录 `docs/` 下的 DESIGN.md / DEV_GUIDE_PSEUDOCODE.md / STAGE1_REVIEW.md / STAGE2_REVIEW.md 是设计过程文档，与现状的差异见 §7，问题见 §8。
> 维护规则：**改代码必须同步改本文档**；本文档与代码冲突时，以代码为准并当场修正文档。

---

## 1. 项目定位

C++17 header-only 零依赖嵌入式底盘运动学库：输入本体系 `Twist (vx, vy, wz)`，输出各轮角速度 `WheelSpeeds`，支持差速 / Mecanum / 全向三类线性底盘。编译期模板（CRTP）派发，无虚函数、零堆、零异常，仅依赖 `<cstdint>` / `<cmath>`。

注意：设计文档中规划的 `speed_limiter.hpp`（STAGE 3）**不在本模块内**——它已演化为独立模块 `control/twist_acc_limiter/`（STATIC 库，见该模块 docs）。

## 2. 实际目录结构

```
kinematics/
├── inc/                        ← 头文件根（设计文档写的是 include/，实际是 inc/）
│   ├── chassis.hpp             ← 对外聚合入口（用户只 include 这一个），7 行
│   ├── contracts.hpp           ← 数据契约：Twist / WheelSpeeds（POD）
│   ├── kinematics.hpp          ← 数学核心：Kinematics<Derived> CRTP 基类 + jacobian_apply + k2PI
│   ├── drive_diff.hpp          ← DiffDrive
│   ├── drive_mecanum.hpp       ← MecanumDrive
│   └── drive_omni.hpp          ← OmniDrive
├── test/test_kinematics.cpp    ← 测试（手写 CHECK 宏，退出码即结果）
├── examples/
│   ├── example.cpp             ← 三底盘 API 调用演示（CMake target: example_diff）
│   └── simulation_demo.cpp     ← 2D 位姿积分仿真（⚠ 未挂 CMake target，见 §8-P3）
├── legacy/                     ← 2024 年 C 语言 nav 参考实现（不属于本库）
├── CMakeLists.txt
└── AGENT.md                    ← 代理协作规则（学习项目，AI 禁改代码）
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
    for (uint8_t i = 0; i < N; ++i) {          // ⚠ 循环上界是 N 不是 wn，见 §8-P1
        out_ws.values_[i] = J[i][0]*t_cmd.vx_ + J[i][1]*t_cmd.vy_ + J[i][2]*t_cmd.wz_;
    }
    return out_ws;
}
```

模板参数 `N` 由数组声明尺寸推导；`wn` 是运行时有效行数。两者只在 OmniDrive（声明 `J[6][3]`、实际 3 行）处不相等——**当前循环遍历到 N 会读写未初始化的 J[3..5] 行**（结果无害但属未定义行为，详见 §8-P1）。

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
- 正运动学：**硬编码 3 轮、γ=0 特例**（伪逆手工展开，√3 取 1.7320508f）：

```
vx = (u2 − u1)/√3        ui = r·ωi
vy = (2u0 − u1 − u2)/3
wz = (u0 + u1 + u2)/(3R)
```

`wn > 3` 或 `γ ≠ 0` 时 inverse 可用（N≤6），**forward 结果错误**（见 §8-P2）。

## 6. 构建与测试（实测）

- CMake ≥ 3.28，C++17。`add_library(kinematics INTERFACE)`（header-only），`target_include_directories` 暴露 `inc/`。
- 三个 target：`test_kinematics`（链接库，`-Wall -Wextra -Werror`）、`example_diff`（= examples/example.cpp，**无** -Werror 选项）；`simulation_demo.cpp` 无 target。
- 测试方法论：锚点（手算有理数如 `50.0f/3.0f`）+ 互逆（forward∘inverse ≈ 恒等）+ 边界（零输入）；容差 1e-5，退出码即结果。
- 手动编译命令（AGENT.md 记载）：`g++ -std=c++17 -Wall -Wextra -Iinc test/test_kinematics.cpp`
- **2026-08-22 实测**：`g++ -std=c++17 -Wall -Wextra -Werror -Iinc test/test_kinematics.cpp` → `ALL PASS`，退出码 0。

## 7. 与根目录设计文档的差异（漂移清单）

| # | 设计文档说 | 代码现实 |
|---|---|---|
| 1 | 目录 `include/kinematics/`、`tests/` | 实际 `inc/`、`test/` |
| 2 | 文件名 `differential_drive.hpp` 等 | 实际 `drive_diff.hpp` / `drive_mecanum.hpp` / `drive_omni.hpp` |
| 3 | 基类名 `Chassis<Derived>`，`chassis.hpp` 是基座 | 实际基类叫 `Kinematics<Derived>`（STAGE2 复盘已修正命名，DESIGN.md 未回改）；`chassis.hpp` 是聚合入口 |
| 4 | STAGE 3 在 kinematics 内做 header-only `speed_limiter.hpp` | 实际落地为 `control/twist_acc_limiter/`（.hpp+.cpp 的 STATIC 库），且不进 chassis.hpp |
| 5 | Omni 逆运动学公式 sin/−cos/−R | 代码为 −sin/+cos/+R（§5.3） |
| 6 | Omni forward 计划"3 轮特例先行，N>3 用一般公式" | 目前只有 3 轮 γ=0 特例，无一般公式 |
| 7 | 统一接口 `inverse<Chassis>(cmd)` 自由函数 | 实际为成员调用 `chassis.inverse_kinematics(cmd)`（DEV_GUIDE 备忘 4 已承认此修正） |
| 8 | 开发计划 STAGE 3"待开发" | 其功能已由 twist_acc_limiter 模块完成 |

## 8. 问题清单（只记录，不修改）

- **P1（正确性隐患）** `inc/kinematics.hpp` `jacobian_apply`：循环上界用编译期 `N` 而非运行时 `wn`。OmniDrive 传 `J[6][3]` + `wn=3` 时，`J[3..5]` 未初始化即被读取（UB；因 `count_=3` 调用方不读后三行，测试碰巧全绿）。建议作者自行改为 `i < wn` 并全零初始化 `J`。
- **P2（功能缺口）** `drive_omni.hpp` `forward_impl` 硬编码 3 轮 γ=0；`wn_`、`gamma_` 成员在 forward 中未使用。构造 `wn>3` 时 forward 输出错误且无断言拦截。
- **P3（构建缺口）** `examples/simulation_demo.cpp` 没有对应 CMake target，不会被构建，是否存在编译错误未知。
- **P4（不一致）** `example_diff` target 未加 `-Wall -Wextra -Werror`，与 `test_kinematics` 不一致。
- **P5（风格）** `contracts.hpp` 的 POD 契约成员用尾下划线命名（`vx_`），与"公共契约"语义存在张力；STAGE1_REVIEW 的 struct-POD 讨论未涉及此点。
- **P6（文档债）** 根 `docs/DESIGN.md` 与 `docs/DEV_GUIDE_PSEUDOCODE.md` 未随 STAGE2 命名修正与 STAGE3 移址回改（§7 全部条目）。
