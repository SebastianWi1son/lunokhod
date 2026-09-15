---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。 kinematics 设计权威；⚠️ 本文已知与代码漂移，清单见 IMPL.md §7。
> 文档体系与写作规则：../../docs/README.md

# Kinematics — 通用平面小车运动学库

## 产品定位

**一个 header、零依赖、拖进工程就能用的 C++17 嵌入式底盘运动学库。**

目标用户：中国大学生机器人竞赛参赛者（电赛、智能车、RoboMaster、Robocon），在 STM32/ESP32 等 MCU 上裸跑 C/C++。

一句话介绍：*一个 header 拖进工程，差速/Mecanum/全向的运动学全部解决。*

## 核心设计原则

1. **最小**：Header-only，零外部依赖（仅 `<cmath>`），单文件或少量文件即可集成
2. **通用**：覆盖线性底盘子集（差速 2WD/4WD、Mecanum 4WD、全向 3/4/N 轮），编译期模板派发
3. **纯数学**：不接触任何硬件（无 I2C、无 PWM、无 GPIO），输入 `(Vx, Vy, ω)` 输出轮速
4. **现代 C++**：C++17 constexpr、static polymorphism、编译期矩阵运算

## 架构

```
kinematics.hpp                    ← 用户唯一需要 #include 的头文件（聚合入口）
├── drive_diff.hpp               ← 差速 2WD/4WD 正/逆运动学
├── drive_mecanum.hpp            ← Mecanum 4WD 正/逆运动学
└── drive_omni.hpp               ← 全向 3/4/N 轮正/逆运动学
```

> **契约（`Twist` / `WheelSpeeds` / `Pose`）不在本库** —— 已提为最底层库 `contracts/`
> （2026-09-15 卫生整理：它不只属 kinematics，odometry 与 twist_acc_limiter 也靠它）。
> **里程计 `odometry.hpp` 也不在本库** —— 已独立为 `odometry/`（它是有状态、可选、
> 非所有用户都需要的组件；本库的对外身份是「纯函数瞬时映射」）。
>
> **限幅器不在本库**。它已演化为独立模块 `control/twist_acc_limiter/`（STATIC 库）
> —— 限幅是**应用层策略**，不是运动学数学（行业惯例放上层：ROS 导航栈 acc_lim、
> 驱动器固件 ramping）。差异清单见 `control/twist_acc_limiter/docs/IMPL.md` 的「与设计文档的差异」。

## 统一接口

```cpp
// 输入：本体系 twist
struct Twist {
    float vx;   // 前进速度 (m/s)
    float vy;   // 侧移速度 (m/s)，差速底盘自动忽略
    float wz;   // 角速度 (rad/s)
};

// 输出：各轮角速度
struct WheelSpeeds {
    float values[MAX_WHEELS];  // rad/s
    uint8_t count;
};

// 统一逆运动学接口
template<typename Chassis>
WheelSpeeds inverse(const Twist& cmd);

// 统一正运动学接口（里程计用）
template<typename Chassis>
Twist forward(const WheelSpeeds& speeds);
```

## 支持的底盘类型及数学公式

### 差速驱动 (Differential Drive)

- 参数：轮距 `w`，轮半径 `r`
- 约束：`Vy ≡ 0`（非完整约束）

```
逆运动学:
  ω_L = (Vx - ω·w/2) / r
  ω_R = (Vx + ω·w/2) / r

正运动学:
  Vx = r·(ω_R + ω_L) / 2
  ω  = r·(ω_R - ω_L) / w
```

### Mecanum 四轮

- 参数：半轮距 `lx`，半轴距 `ly`，轮半径 `r`
- `Vy` 自由（完整约束）

```
逆运动学矩阵 (X型排列, 滚子±45°):
  [ω_fl]   [1  -1  -(lx+ly)] [Vx]
  [ω_fr] = [1   1   (lx+ly)]·[Vy] · (1/r)
  [ω_rl]   [1   1  -(lx+ly)] [ω ]
  [ω_rr]   [1  -1   (lx+ly)]

正运动学 (Moore-Penrose 伪逆):
  4轮速度 → (Vx, Vy, ω)
```

### 全向 N 轮

- 参数：轮心距 `R`，轮数 `n`，首轮偏移角 `γ`，轮半径 `r`

```
第 i 轮:
  ω_i = [sin((i-1)·2π/n + γ)·Vx
       - cos((i-1)·2π/n + γ)·Vy
       - R·ω] / r
```

## SpeedLimiter（可选附赠，非核心）

Twist 空间加速度限幅器（斜坡发生器）：上游指令可任意跳变，输出每通道最多以 `acc·dt` 的斜率平滑变化，保运动学一致性。**不在 chassis.hpp 聚合入口内**，需要时单独 include。

```cpp
SpeedLimiter limiter(1.0f, 0.5f, 2.0f);   // vx/vy/wz 三通道加速度上限

auto [out, vx_lim, vy_lim, wz_lim] = limiter.limit(cmd, dt);  // 平滑限幅 + 每通道饱和标志
```

- v1 范围：**每通道加速度斜坡**（限变化率，保 Twist 几何一致性）。不含速度上限、Jerk、联合约束（功率/安全层职责，属后续）
- 为什么是小礼包：限幅是应用层策略，行业惯例放上层（ROS 导航栈 acc_lim、驱动器固件 ramping）
- 完整设计决策与伪代码：`DEV_GUIDE.md` STAGE 3（同目录）

## 刻意不覆盖的范围

| 不覆盖 | 原因 |
|--------|------|
| **Ackermann（汽车转向）** | **可达的 Twist 子空间比差速更小（缺「原地转」），且逆映射在 `vx→0` 有奇点** —— 与「互逆」验收锚点冲突。见**说明一** |
| **传感器硬件操作** | 破坏"纯数学、零依赖"原则 |
| **PID 控制器** | 市面上已有 N 个成熟的 PID 库 |
| **轨迹规划/路径跟踪** | 上层逻辑，不属于底盘运动学层 |

### 说明一：「不能横移」不是排除理由（2026-09-15 修正）

本表原先记 Ackermann 为「非线性 tan(δ)，**输入空间不完整**」—— **这个说法不准确。**

**差速也不能横移。** 契约里写着「`vy` 差速底盘自动忽略」，
而 `test/test_kinematics.cpp` 的 `test_diff()` 明确**断言 `back.vy_ == 0`**（承认它被丢弃）。
**所以「丢 `vy`」是设计进去的、被明确接受的，不是缺陷。**

**真正的分界是「能不能原地转」**：

| | 可达 Twist 子空间 | 逆映射 | 原地转（`vx=0, wz≠0`） |
|---|---|---|---|
| **差速** | `{vy = 0}` | 线性、**处处可逆** | ✅ ***`test_diff` 的锚点就是它*** |
| **Ackermann** | `{vy = 0, 且 wz≠0 时 vx≠0}` | **非线性 + `vx→0` 奇点** | ❌（`δ = atan(wz·L/vx)` 发散，车不能原地转） |

→ **排除 Ackermann 的本质理由**：它的可达子空间比差速更小、逆映射有奇点，
**与「互逆」这条验收锚点冲突**。要收它就得**换一套验收标准**。

### 说明二：Swerve 的排除理由**站不住**，已转入待办（2026-09-15）

`THEORY.md` §2.4 自己给出的公式：

```
φ_i = atan2(Vy + ω·x_i,  Vx − ω·y_i)
v_i = sqrt((Vx − ω·y_i)² + (Vy + ω·x_i)²)
```

**这是标准的一对一瞬时映射，零隐藏状态。**

原表说的「有跨帧状态（180° 翻转管理）」其实是：`(φ,v)` 与 `(φ+π,−v)` 等效，
选哪个只为了**让舵机少转** —— **那是轮控层的优化，不是运动学**。

**真正的障碍是输出契约太窄**：`WheelSpeeds` 只有一个 `float values_[6]`，
而 Swerve 每模块要出**两个量**（角度 + 速度）。

→ **Swerve 是「契约太窄」，Ackermann 是「抽象不成立」。** 前者值得做，后者不值得。
详见 `../../docs/TODO.md` P21。

### 说明三：履带 / skid-steer 不需要新代码（2026-09-15 补）

**履带的 J 矩阵与差速完全相同**（等价于两个差速轮组）。
差别只在**打滑** —— 那是感知层的事，不是运动学。
所以本库**不需要** `drive_tracked.hpp`；需要的是**在文档里说清楚**（就是这里）。
（原先本库一个字没提履带，读者会以为漏了。）

## 竞品与差异化

| 竞品 | 缺陷 | 我们的优势 |
|------|------|-----------|
| ROS2 diff_drive_controller | 绑定 ROS2 生态，仅差速 | Header-only，多底盘，零依赖 |
| linorobot/kinematics | 绑定 Arduino，仅差速+Mecanum | 支持全向 N 轮，C++17 |
| RT-Thread rt-robot | 绑定 RTOS，C 语言 | Header-only C++17，无平台绑定 |
| WPILib kinematics | 依赖 Eigen+units+框架 | 轻量级，嵌入式友好 |

**核心差异化**：市面没有"零依赖 header-only + 全部线性底盘 + 嵌入式可用"的运动学库。

## 规模

> 本节原为“**预期规模**”（含 Stars 预测），2026-09-14 按 `../../docs/TODO.md` P13 **删除**。
>
> **为什么删**：B 类事实文档里**不许有主观预测** —— 预测必然过期，而且会被后来的读者当真。
> 被删的内容里，\<300 行核心代码 / 5-6 个头文件已被实测证伪（实际分别是 1071 行 / 7 个）。
>
> 依赖与标准（仅 `<cmath>` / `<cstdint>`、C++17）已在「核心设计原则」；
> 实际行数、文件清单属于 C 类状态，见 [`IMPL.md`](IMPL.md)。

## 项目结构

```
kinematics/
├── inc/                           ← 头文件（纯 header，无 .cpp）
│   ├── kinematics.hpp             ← 用户唯一入口（聚合）
│   ├── chassis.hpp                ← 底盘门面（统一接口）
│   ├── drive_diff.hpp             ← 差速 2WD/4WD
│   ├── drive_mecanum.hpp          ← Mecanum 4WD
│   └── drive_omni.hpp             ← 全向 3/4/N 轮
├── test/
│   └── test_kinematics.cpp        ← 行为锚点测试
├── examples/
│   ├── example.cpp
│   └── simulation_demo.cpp        ← 全链仿真（逆解 → 轮子 → 正解 → 里程计）
├── docs/                          ← DESIGN / THEORY / DEV_GUIDE / IMPL + log/
└── legacy/                        ← 2024 年 C 语言巡线实现（只读参考，不许改）
```

> **2026-09-14 修正**：本表原先写 `include/kinematics/`、`tests/`、`README.md`（均不存在），
> 以及已改名 / 已迁出的旧文件名 —— 与实际全不符。见 `IMPL.md` §7 漂移清单。
> **2026-09-15 修正**：`contracts.hpp` 与 `odometry.hpp` 及其测试/工具/文档已分别迁至
> `../contracts/` 与 `../odometry/`；本库的 `test/` `tools/` `inc/` 不再含它们。

## 开发计划

| 阶段 | 内容 | 状态 |
|------|------|:--:|
| 1 | `contracts.hpp` + `drive_diff.hpp`（当时叫 differential_drive） | 已完成 |
| 2 | `drive_mecanum.hpp` + `drive_omni.hpp` | 已完成 |
| 3 | 加速度限幅 → **迁出为独立模块** `control/twist_acc_limiter/` | 已完成（非本库） |
| 4 | 单元测试 + 文档 + 示例 | 已完成 |
| 5 | PlatformIO / Arduino 库注册 | 待开始 |
| 6 | `odometry.hpp` 轮速里程计（当时未规划） | 已完成（2026-09-14）→ **2026-09-15 迁出为独立库 `odometry/`** |

## 旧项目关系

本目录与 `../legacy/` 的关系：
- `legacy/` 是 2024 年第一次制作灰度+IMU 循迹小车时的 nav 控制器（C 语言，硬编码差速底盘）
- 本库是旧项目的**理念升级**：从"一种底盘的一种控制方式"升级为"通用底盘运动学组件"
- 旧代码中的 `nav_line.c` 核心公式 `*L = speed + steer; *R = speed - steer;` 在本库中对应 `differential_drive.hpp` 的逆运动学
