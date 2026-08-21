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
kinematics.hpp                    ← 用户唯一需要 #include 的头文件
├── types.hpp                    ← Twist, WheelSpeeds, 单位定义
├── differential_drive.hpp       ← 差速 2WD/4WD 正/逆运动学
├── mecanum_drive.hpp            ← Mecanum 4WD 正/逆运动学
├── omni_drive.hpp               ← 全向 3/4/N 轮正/逆运动学
└── speed_limiter.hpp            ← 速度/加速度/Jerk 三级限幅器
```

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

## SpeedLimiter

独立的速度/加速度/Jerk 三级限幅器。对 `Vx`、`Vy`、`ω` 分别限幅。

```cpp
SpeedLimiter limiter;
limiter.setMaxVel({2.0f, 1.0f, 3.14f});  // Vx, Vy, ω 上限
limiter.setMaxAcc({1.0f, 0.5f, 2.0f});   // 加速度上限

auto limited = limiter.step(target, dt);  // 平滑限幅
```

## 刻意不覆盖的范围

| 不覆盖 | 原因 |
|--------|------|
| **Ackermann (汽车转向)** | 非线性 tan(δ)，输入空间不完整 |
| **Swerve Drive** | 非线性 atan2 + sqrt，有跨帧状态（180°翻转管理） |
| **传感器硬件操作** | 破坏"纯数学、零依赖"原则 |
| **PID 控制器** | 市面上已有 N 个成熟的 PID 库 |
| **轨迹规划/路径跟踪** | 上层逻辑，不属于底盘运动学层 |

## 竞品与差异化

| 竞品 | 缺陷 | 我们的优势 |
|------|------|-----------|
| ROS2 diff_drive_controller | 绑定 ROS2 生态，仅差速 | Header-only，多底盘，零依赖 |
| linorobot/kinematics | 绑定 Arduino，仅差速+Mecanum | 支持全向 N 轮，C++17 |
| RT-Thread rt-robot | 绑定 RTOS，C 语言 | Header-only C++17，无平台绑定 |
| WPILib kinematics | 依赖 Eigen+units+框架 | 轻量级，嵌入式友好 |

**核心差异化**：市面没有"零依赖 header-only + 全部线性底盘 + 嵌入式可用"的运动学库。

## 预期规模

| 指标 | 预期 |
|------|------|
| 核心代码量 | < 300 行 |
| 头文件数 | 5-6 个 |
| 编译依赖 | 仅 `<cmath>` |
| C++ 标准 | C++17 |
| 预期 Stars (2年) | 100-250 |
| 乐观 Stars (5年) | 300-600 |

## 项目结构

```
kinematics/
├── docs/
│   ├── DESIGN.md              ← 本文件
│   └── THEORY_AND_REFERENCE.md  ← 理论参考资料
├── include/
│   └── kinematics/
│       ├── kinematics.hpp     ← 用户唯一入口
│       ├── types.hpp          ← 公共类型定义
│       ├── differential_drive.hpp
│       ├── mecanum_drive.hpp
│       ├── omni_drive.hpp
│       └── speed_limiter.hpp
├── examples/
│   ├── differential_drive_example.cpp
│   ├── mecanum_drive_example.cpp
│   └── dual_sensor_fusion.cpp ← SensorFusion 模板（见 sensor_fusion/docs/）
├── tests/
│   └── test_kinematics.cpp
└── README.md
```

## 开发计划

| 阶段 | 内容 | 状态 |
|------|------|:--:|
| 1 | `types.hpp` + `differential_drive.hpp` | 待开始 |
| 2 | `mecanum_drive.hpp` + `omni_drive.hpp` | 待开始 |
| 3 | `speed_limiter.hpp` | 待开始 |
| 4 | 单元测试 + 文档 + 示例 | 待开始 |
| 5 | PlatformIO / Arduino 库注册 | 待开始 |

## 旧项目关系

本目录与 `../legacy/` 的关系：
- `legacy/` 是 2024 年第一次制作灰度+IMU 循迹小车时的 nav 控制器（C 语言，硬编码差速底盘）
- 本库是旧项目的**理念升级**：从"一种底盘的一种控制方式"升级为"通用底盘运动学组件"
- 旧代码中的 `nav_line.c` 核心公式 `*L = speed + steer; *R = speed - steer;` 在本库中对应 `differential_drive.hpp` 的逆运动学
