---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。 理论推导与开源参考。
> 文档体系与写作规则：../../docs/README.md

# 通用平面小车控制器 — 理论基础与开源参考

> 本文档为从零重构 C++ 通用平面小车控制器提供理论基础。核心思想：**上层只下发 (Vx, Vy, ω) 三量，底层按底盘类型做运动学分解**。

---

## 1. 数学模型总览

### 1.1 统一范式：Unicycle Model（独轮/单轮模型）

所有非完整约束平面机器人都可以归约到 canonical unicycle model：

```
ẋ = Vx · cos(θ)
ẏ = Vx · sin(θ)
θ̇ = ω
```

其中 `(x, y, θ)` 是世界坐标系下的位姿，`Vx` 是本体系前进线速度，`ω` 是本体系角速度。

**非完整约束**（无法侧滑）：`ẋ·sin(θ) - ẏ·cos(θ) = 0`，即本体系中 `Vy ≡ 0`。

参考来源：
- Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*
- [ROS2 Control: Mobile Robot Kinematics](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html)

### 1.2 完整约束 vs 非完整约束

| 类型 | 约束 | Vy | 典型底盘 | 轮数 |
|------|------|:--:|---------|:----:|
| Nonholonomic (非完整) | 不可侧滑 | ≡ 0 | 差速驱动、Ackermann、自行车 | 2~4 |
| Holonomic (完整) | 无约束 | 自由 | 全向轮 (omni)、麦克纳姆轮 (mecanum) | 3~4 |

**统一接口设计原则**：无论哪种底盘，控制器输入都应该是 `cmd = (Vx, Vy, ω)`，由运动学层做映射。

---

## 2. 各类底盘的正/逆运动学公式

### 2.1 差速驱动 (Differential Drive) — 你最熟悉的

**物理参数**：
- `w` = 轮距 (两驱动轮中心间距)
- `r` = 轮半径
- `v_L, v_R` = 左右轮线速度

```
Forward Kinematics (轮速 → 本体系速度):
  Vx   = (v_R + v_L) / 2
  Vy   = 0              ← 非完整约束
  ω    = (v_R - v_L) / w

Inverse Kinematics (本体系速度 → 轮速):
  v_L  = Vx - ω·w/2
  v_R  = Vx + ω·w/2

  角速度形式 (ω_L, ω_R 为轮子角速度):
  ω_L  = (Vx - ω·w/2) / r
  ω_R  = (Vx + ω·w/2) / r
```

**控制空间形状**：菱形（单个轮速有限幅时）。当 `ω = Vx` 可以纯原地旋转。

这是旧项目 nav 控制器中使用的模型。旧代码中的：
```c
*L = speed + steer;  // steer = PID + FF output
*R = speed - steer;
```
等价于上述逆运动学，其中 `speed = Vx`, `steer = ω·w/2`。

**多轮差速**：同侧多轮共享同一速度命令，多轮带来的横向摩擦视为被动的 scrub 自由度。

### 2.2 麦克纳姆轮 (Mecanum Drive) — 四轮全向

**关键参数**：
- `l` = 半轮距（左右轮距的一半）
- `b` = 半轴距（前后轮距的一半）

**逆运动学矩阵**（`X` 型排列，滚子 ±45°）：

```
[v_fl]   [ 1  -1  -(l+b) ] [Vx]
[v_bl] = [ 1   1  -(l+b) ]·[Vy] · 1/r
[v_br]   [ 1  -1   (l+b) ] [ω ]
[v_fr]   [ 1   1   (l+b) ]
```

其中 `fl=前左, bl=后左, br=后右, fr=前右`。

**正运动学**（4→3，Moore-Penrose 伪逆）：

```
[Vx]   r [ 1   1   1   1 ] [v_fl]
[Vy] = - [ -1   1  -1   1 ]·[v_bl]
[ω ]   4 [ -1/(l+b)  -1/(l+b)  1/(l+b)  1/(l+b) ] [v_br]
                                                   [v_fr]
```

参考：[Mobile Robot Kinematics for FTC](https://rr.brott.dev/papers/Mobile_Robot_Kinematics_for_FTC.html)

### 2.3 全向轮 (Omni-wheel, 3 轮)

**N 轮全向通用逆运动学矩阵**（ROS2 规范）：

设 R = 轮心到中心的距离，γ = 第一轮相对 x_b 的角度偏移，θ = 2π/n 为轮间夹角。

第 i 个轮 (i = 0..n-1) 的角速度：

```
ω_i = [ sin((i-1)θ + γ) · Vx
      - cos((i-1)θ + γ) · Vy
      - R · ω ] / r
```

**3 轮简化**（120° 均布，γ=0）：

```
ω_1 = (-sin30° · Vx - cos30° · Vy - R · ω) / r
ω_2 = (-sin30° · Vx + cos30° · Vy - R · ω) / r
ω_3 = (Vx - R · ω) / r
```

### 2.4 Swerve Drive（独立转向驱动模块）

每个模块 i 在位置 `(x_i, y_i)`：

```
φ_i = atan2(Vy + ω·x_i,  Vx - ω·y_i)     ← 转向角
v_i = sqrt((Vx - ω·y_i)² + (Vy + ω·x_i)²) ← 轮线速度
```

最灵活的构型，但每个模块需要两个电机（驱动+转向）。

### 2.5 自行车/Ackermann（类汽车模型）

由前轮转角 `δ` 控制转弯：

```
Vx = v                     ← 后轮前进速度
ω  = v · tan(δ) / L        ← L = 轴距
```

约束：存在最小转弯半径 `R_min = L / tan(δ_max)`，导致控制空间呈蝴蝶结形。

---

## 3. 控制架构设计原则

### 3.1 分层架构（参考 ROS2 ros2_control）

```
     高层规划 / 遥控
          │
    cmd = (Vx, Vy, ω)     ← 统一接口 (Twist/ChassisCommand)
          │
    ┌─────┴─────┐
    │ Velocity   │        ← 速度/加速度/Jerk 限幅器
    │ Limiter    │
    └─────┬─────┘
          │
    ┌─────┴─────┐
    │  Inverse   │        ← 底盘运动学逆解（矩阵乘法）
    │ Kinematics │
    └─────┬─────┘
          │
    [ω_1, ω_2, ..., ω_n]  ← 各轮目标角速度/线速度
          │
    ┌─────┴─────┐
    │  Wheel PID │         ← 每轮独立速度闭环
    └─────┬─────┘
          │
      电机驱动
```

### 3.2 关键模块

| 模块 | 职责 | 参考 |
|------|------|------|
| **Command Interface** | 接收统一 `(Vx, Vy, ω)` 命令 | `geometry_msgs/Twist` |
| **SpeedLimiter** | 对 Vx, Vy, ω 分别施加速度/加速度/Jerk 限制 | ROS2 `SpeedLimiter` |
| **Kinematics Solver** | 底盘逆运动学：`(Vx,Vy,ω) → [wheel_speeds]` | 矩阵乘法 |
| **Odometry** | 正运动学 + 里程计积分：`[wheel_speeds] → (x,y,θ)` | Runge-Kutta / Euler |
| **Wheel PID** | 每轮速度/位置闭环 | PID 控制器 |

### 3.3 限幅 (Limiting) 策略

必须对统一命令 `(Vx, Vy, ω)` 限幅之后再做运动学分解。如果先分解再限幅，会导致底盘运动扭曲。

ROS2 diff_drive_controller 使用独立的 `SpeedLimiter` 对 `Vx` 和 `ω` 分别做：
- 速度限幅 (min/max velocity)
- 加速度限幅 (max acceleration, max deceleration)
- Jerk 限幅 (min/max jerk)

---

## 4. 开源参考项目清单

### 4.1 工业级（ROS2，最成熟）

| 项目 | 说明 | Stars | 语言 |
|------|------|------:|------|
| **[ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)** — `diff_drive_controller` | ROS2 官方差速控制器，架构最规范。逆运动学核心仅 2 行：`v_left = (lin - ang * sep/2) / r` | 500+ | C++ |
| **[ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)** — `tricycle_controller` | Ackermann/自行车模型控制器 | — | C++ |
| **[ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)** — `ackermann_steering_controller` | Ackermann 转向控制器 | — | C++ |

> 🔑 **diff_drive_controller 是最值得精读的参考**，其架构（SpeedLimiter → IK → wheel_cmd）是通用范式。

### 4.2 RoboMaster / 竞赛级

| 项目 | 说明 | 语言 |
|------|------|------|
| **[rm-controls/rm_controllers](https://github.com/rm-controls/rm_controllers)** | RoboMaster 底盘控制器，支持 Mecanum + Swerve，带功率限制。`OmniController` + `SwerveController` 都接受 Twist 输入 | C++ |
| **[gdut-robocon/rc_controllers](https://github.com/gdut-robocon/rc_controllers)** | 全国大学生机器人大赛 Robocon 控制器，支持 Omni + Swerve + Chassis 变换 | C++ |

> 🔑 rm_controllers 的 `OmniController` 是一个非常好的参考，支持四轮 mecanum 和全向的 YAML 配置化逆运动学。

### 4.3 Arduino / 嵌入式

| 项目 | 说明 | Stars |
|------|------|------:|
| **[linorobot/kinematics](https://github.com/linorobot/kinematics)** | Arduino 运动学库，支持差速 2WD/4WD + Mecanum。API: `getRPM(Vx, Vy, ω)` → 各轮 RPM | 71 |
| **[sahilrajpurkar03/robot-drive-systems](https://github.com/sahilrajpurkar03/robot-drive-systems)** | AVR 单片机驱动系统集：2轮差速 + 3轮全向 + 4轮全向，带逆运动学公式 | — |

> 🔑 linorobot 的 API 设计（`getRPM(Vx, Vy, ω)`）是小车控制器最直观的接口。

### 4.4 全向专项

| 项目 | 说明 | 语言 |
|------|------|------|
| **[yukimakura/ros2_controller_for_holonomic_wheels](https://github.com/yukimakura/ros2_controller_for_holonomic_wheels)** | ROS2 全向轮控制器，支持 3 轮 omni + 4 轮 omni/mecanum | C++ |
| **[mateusmenezes95/omnidirectional_controllers](https://github.com/mateusmenezes95/omnidirectional_controllers)** | ROS2 三全向轮控制器，含完整运动学文档 | C++ |
| **[devrt/offset-diff-drive-controller](https://github.com/devrt/offset-diff-drive-controller)** | 偏置双差速驱动实现全向（用非全向轮实现全向的巧妙方案） | C++ |
| **[t413/omnictrl](https://github.com/t413/omnictrl)** | C++ 嵌入式全向轮控制器，CAN 总线 + ODrive + ESP-NOW 遥控 | C++ |

### 4.5 理论与文档

| 资源 | 说明 |
|------|------|
| [ROS2 Control: Mobile Robot Kinematics](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html) | ROS2 官方运动学文档，涵盖差速/全向/Swerve/Tricycle |
| [Mobile Robot Kinematics for FTC](https://rr.brott.dev/papers/Mobile_Robot_Kinematics_for_FTC.html) | 从基础推导差速/Mecanum/Swerve 完整逆运动学矩阵 |
| [UCR Robotics: Kinematics and Control](https://ucr-robotics.readthedocs.io/en/latest/tbot/moRbt.html) | 状态空间模型 + 差速推导 + 反馈控制基础 |
| [Modern Robotics (Lynch & Park)](https://modernrobotics.northwestern.edu/) | 经典教材，Ch13.3 非完整约束移动机器人运动学 |
| [roboticsbook.org: DDR Motion Model](https://www.roboticsbook.org/S52_diffdrive_actions.html) | 差速驱动运动模型 Python 代码示例 |
| [MATLAB: Mobile Robot Kinematics](https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html) | MATLAB 官方运动学文档，含 unicycle/bicycle/diff-drive/Ackermann |

---

## 5. 重构建议：C++ 最小核心架构

### 5.1 设计目标

- **Header-only 优先**：运动学矩阵是编译期常量，用 `constexpr` 和内联函数
- **静态多态**：用 `template` + `CRTP` 或 `concepts` (C++20) 做底盘类型派发
- **零依赖**：除 `<cmath>` 外无外部依赖，适合嵌入式
- **统一接口**：输入 `(Vx, Vy, ω)` → 输出 `[wheel_speeds]`

### 5.2 建议的类层次

> ⚠ **2026-09-15：本节是早期提案，与现状有出入，勿当权威。** 现状以
> `DESIGN.md`（架构）与 `IMPL.md`（代码地图）为准。差异：
>
> | 提案 | 现状 |
> |---|---|
> | 基类 `ChassisKinematics<Derived>` | 实际叫 `Kinematics<Derived>`（STAGE2 复盘已改名） |
> | `Omni3Wheel` / `Omni4Wheel` 两个类 | 实际是**一个通用 `OmniDrive`**（N 轮 + 任意 γ） |
> | `SwerveDrive` | **不在计划内**，原因是**输出契约太窄**而非「有状态」（见 `DESIGN.md` 说明二） |
> | `SpeedLimiter` | 已迁出为独立库 `control/twist_acc_limiter/` |
> | `ChassisController<Kinematics>` | ❌ **从未实现** —— 装配逻辑至今住在消费方，见 `../../docs/TODO.md` P20 |

```
ChassisKinematics<Derived>          ← CRTP 基类，定义统一接口
├── DifferentialDrive              ← 差速 2/4 轮
├── MecanumDrive                   ← 麦克纳姆 4 轮
├── Omni3Wheel                     ← 全向 3 轮
├── Omni4Wheel                     ← 全向 4 轮
└── SwerveDrive                    ← Swerve N 模块

SpeedLimiter                        ← 独立的速度/加速度/Jerk 限幅器

ChassisController<Kinematics>       ← 模板化控制器
  ├── void setCommand(Vx, Vy, ω)
  ├── void update(dt)
  └── WheelSpeeds getWheelSpeeds()
```

### 5.3 逆运动学接口设计（参考 linorobot）

```cpp
struct Twist {
    float vx;   // m/s, 前进
    float vy;   // m/s, 侧移 (全向底盘)
    float wz;   // rad/s, 角速度
};

struct WheelSpeeds {
    float values[MAX_WHEELS];  // rad/s 或 RPM
    uint8_t count;
};

// 统一接口
WheelSpeeds inverse_kinematics(const Twist& cmd);
Twist forward_kinematics(const WheelSpeeds& speeds);   // 里程计
```

### 5.4 与旧项目的对比

| 维度 | 旧项目 (nav.c) | 新设计 |
|------|---------------|--------|
| 底盘假设 | 固定差速 | 可配置多种底盘 |
| 接口 | 直接输出 L/R RPM | 统一 `(Vx, Vy, ω)` → 轮速 |
| 运动学 | 隐式（藏在 steer 公式里） | 显式逆运动学矩阵 |
| 限幅 | 无 | SpeedLimiter 三级限幅 |
| 语言 | C | C++ (C++17/20) |
| 可复用性 | 仅本项目 | 通用开源组件 |

---

## 6. 关键数学补充

### 6.1 全向轮通用矩阵推导

N 个全向轮，每轮 i 在位置 (x_i, y_i)，固定安装角 φ_i（轮面法线方向），滚子角 γ_i（通常为 0 或 π/2）。

每轮驱动方向（垂直于滚子轴）的单位向量：

```
d_i = [cos(φ_i + γ_i), sin(φ_i + γ_i)]^T
```

第 i 轮与底盘中心的速度关系（刚体运动）：

```
v_i = d_i^T · [Vx - ω·y_i,  Vy + ω·x_i]^T
```

合并为矩阵：

```
[v_1]   [ d_1^T   d_1^T·[-y_1, x_1]^T ] [Vx]
[v_2] = [ d_2^T   d_2^T·[-y_2, x_2]^T ]·[Vy]
[...]   [  ...              ...         ] [ω ]
```

这就是 **Jacobian 矩阵 J**。逆运动学用 J 直接乘；正运动学用 `pinv(J)`。

### 6.2 控制空间（Control Set）的可视化

对于差速底盘，假设每个轮速 ∈ [-1, 1]，则 `(Vx, ω)` 可达域是一个菱形：
- 纯前进: `(Vx=1, ω=0)` → 两轮同速前进
- 纯旋转: `(Vx=0, ω=2/w)` → 两轮反向
- 弧线: 在菱形内部任意点

对于 mecanum 底盘，`(Vx, Vy, ω)` 的可达域是一个八面体（4 轮×2 方向约束）。

---

## 7. 参考资料的优先级建议

精读顺序（从最实用开始）：

1. **[ROS2 diff_drive_controller 源码](https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp)** — 直接抄 `update()` 函数中的 IK 公式和 SpeedLimiter 调用模式
2. **[ROS2 Mobile Robot Kinematics 文档](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html)** — 理解每种底盘的 IK/FK 矩阵
3. **[FTC Kinematics 论文](https://rr.brott.dev/papers/Mobile_Robot_Kinematics_for_FTC.html)** — 从第一性原理推导，数学严谨
4. **[linorobot/kinematics](https://github.com/linorobot/kinematics)** — 看简洁的嵌入式 API 设计
5. **[rm_controllers](https://github.com/rm-controls/rm_controllers)** — 看完整的竞赛级实现

---

*文档生成时间：2025-07，基于 exa 网络搜索 + 旧项目代码阅读。*
