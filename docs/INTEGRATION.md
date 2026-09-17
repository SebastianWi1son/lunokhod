---
class: fact
generated: false
---
> **类：B 事实** —— **下游接入指南**：怎么把 5 个库 + 姿态库装成一条链。
> 契约与决策的唯一来源是各组件的 `docs/DESIGN.md`；**指路**：装配层 [`../chassis_loop/docs/DESIGN.md`](../chassis_loop/docs/DESIGN.md) §5.2。
> 本文只讲**怎么装、怎么接**，不重复设计理由。待办见 [`TODO.md`](TODO.md) **P31**。

# INTEGRATION.md — 下游接入指南（一份完整调用）

## 0. 一句话

**你只需要实现两个函数**（读编码器、写 effort），把四个积木按序拼起来，然后**每拍调一次** `tick(dt, now)`。

```
        ┌─────────────── 你写（平台层 / HAL）───────────────┐
        │  read_encoder(id, dt) → rad/s      write_effort(id, effort) → 硬件  │
        └───────────────────────┬───────────────────────────┘
                                │
  上层 ──► Twist ──► [ lunokhod::chassis_loop::ChassisLoop<WheelSet> ] ──► pose / yaw_odo
                     限幅 → 目标 → 执行 → 测量 → 正解 → 里程计
```

## 0.1 名字怎么念（2026-09-17 定案）

所有类型都在 **`lunokhod::`** 之下，**子命名空间 = 组件名**；**跨库共享的数据在根**：

| 你会写到的 | 全限定名 |
|---|---|
| 车体三量 / 轮速 / 位姿 | `lunokhod::Twist` · `lunokhod::WheelSpeeds` · `lunokhod::Pose` |
| 底盘几何 | `lunokhod::kinematics::MecanumDrive`（差速 `DiffDrive` / 全向 `OmniDrive`） |
| 装配层 | `lunokhod::chassis_loop::WheelLoop` · `ChassisLoopConfig` · `WheelSetConfig` |
| 轮控参数与 IO | `lunokhod::wheel::PIDConfig` · `MeasureSpeedFn` · `SetEffortFn` |
| 里程计与记录 | `lunokhod::odometry::Odometry` · `OdometrySample` |

消费方两种写法都行：**写全限定名**，或文件内 `using namespace lunokhod;`（本文示例用后者，另附全限定名注释）。

## 1. 谁写什么

| 谁 | 写什么 |
|---|---|
| **你**（平台层） | `read_encoder` / `write_effort` 两个函数（§2）—— **这是全部的平台层契约** |
| **你**（应用层） | 选底盘、填两份配置、装配、主循环（§3 / §4） |
| **库里已有** | 限幅（`twist_acc_limiter`）· 几何（`kinematics`）· 轮控 S 曲线 + PID（`actuator/wheel`）· 位姿积分与记录（`odometry`）· 编排与顺序（`chassis_loop`） |
| **库外** | 姿态融合（`foucault`）：**只收标量**，见 §5 |

## 2. 平台层的两个函数（唯一契约）

```cpp
lunokhod::wheel::MeasureSpeedFn = float (*)(uint8_t motor_id, float dt);       // 返回：该轮实测转速 [rad/s]
lunokhod::wheel::SetEffortFn    = void  (*)(uint8_t motor_id, int16_t effort);  // effort ∈ [-1000, 1000]
```

| 要定的 | 约定 | 为什么 |
|---|---|---|
| **单位** | `read_encoder` 返回 **rad/s**（与 `wheel_radius` 同一单位制） | 目标是「轮角速度」，不是「米/秒」也不是「计数」 |
| **符号** | 正 = 该轮正转 = **车体前进方向** | 逆解/正解/里程计全按这个符号；反了会出现"跑得对、位姿反" |
| **effort 量程** | ±1000 = `lunokhod::wheel::PIDConfig::limit_out_` 的值 | 库里不假设你的 PWM 是 8 位还是 16 位；**量程由你在配置里定** |
| **`dt` 是提示** | 可用于差分/滤波，**不保证非零** | 时间基的唯一来源是 `tick(dt, now)`，平台层不许自己取时间 |
| **`motor_id` 范围** | `0..5`；**只有前 `count_` 个会被调到** | 未用到的轮子**永远不会**被 `apply()` 碰（不会写 PWM、不会调 `read_encoder`） |

## 3. 完整装配（本节代码已实测：`-Wall -Wextra -Werror` 零告警，且真跑出位姿）

```cpp
// ============ 最小可用装配：一份能编能跑的完整调用 ============
#include <cstdio>                 // 仅示例打印用（库本身零依赖）
#include "wheel_set.hpp"          // 执行器组（它自带 chassis_loop.hpp）
#include "drive_mecanum.hpp"      // 底盘几何（差速用 drive_diff.hpp，全向用 drive_omni.hpp）

// 命名空间（2026-09-17）：全仓类型都在 `lunokhod::` 之下（规则见 lunokhod/AGENTS.md §3）。
// 消费方两种写法都行：① 像下面这样 using ② 写全限定名，如
//   lunokhod::chassis_loop::WheelLoop<lunokhod::kinematics::MecanumDrive> loop(...);
using namespace lunokhod;                    // Twist / Pose / WheelSpeeds
using namespace lunokhod::kinematics;        // MecanumDrive
using namespace lunokhod::chassis_loop;      // ChassisLoopConfig / WheelSetConfig / WheelLoop
using namespace lunokhod::odometry;          // OdometrySample（记录导出口的入参）
using namespace lunokhod::wheel;             // PIDConfig

enum { kWheelCount = 6 };         // 契约容量（WheelSpeeds::values_ 就是 6）

// ---------- ① 平台层：你唯一需要实现的两个函数 ----------
static float   g_speed[kWheelCount] = {0.0f};   // 编码器测得的轮转速 [rad/s]
static int16_t g_effort[kWheelCount] = {0};     // 最近一次写出的 effort

static float read_encoder(uint8_t motor_id, float dt) {
    (void)dt;                     // dt 给你做差分/滤波用；本实现读的是驱动侧的测速寄存器
    return g_speed[motor_id];     // 正 = 该轮正转（= 车体前进方向）
}
static void write_effort(uint8_t motor_id, int16_t effort) {
    g_effort[motor_id] = effort;  // effort ∈ [-1000, 1000]，正 = 正转
}

// PC 替身：一阶跟随，只为让示例在 PC 上跑出非零位姿（真机换成真硬件）
static void sim_step(float dt) {
    (void)dt;
    for (uint8_t i = 0; i < kWheelCount; ++i) {
        const float target = 0.01f * static_cast<float>(g_effort[i]);   // effort → 目标转速
        g_speed[i] += (target - g_speed[i]) * 0.05f;                     // 一阶滞后（时间常数 ≈ 20 拍）
    }
}

// ---------- ② 数据导出（可选）：每拍一条记录 ----------
static void on_sample(void* ctx, const OdometrySample& s) {
    static int n = 0;
    if (ctx != nullptr && (n++ % 400) == 0) {
        std::printf("tick %u  cmd.vx=%.3f  twist.vx=%.3f  pose=(%.3f, %.3f)  wheel0=%.2f\n",
                    s.tick_, s.cmd_.vx_, s.twist_.vx_, s.pose_.x_, s.pose_.y_, s.ws_.values_[0]);
    }
}

int main() {
    // ---- 参数：两份配置，各归各家 ----
    ChassisLoopConfig lcfg;                     // 编排层：限幅 + 里程计标定
    lcfg.acc_vx_ = 1.5f; lcfg.acc_vy_ = 1.5f; lcfg.acc_wz_ = 4.0f;

    WheelSetConfig wcfg;                        // 执行器组：轮控
    wcfg.pid_ = PIDConfig{}.kp(8.0f).ki(1.2f).kd(0.0f)
                            .limit_out(1000.0f).limit_i(300.0f).max_rate_out(4000.0f);
    // wcfg.planner_ 用默认（两级 LPF 后的 S 曲线）

    // ---- 装配：几何 + 执行器组 + 编排层 ----
    MecanumDrive chassis(0.10f, 0.12f, 0.03f);  // 半轴距 / 半轮距 / 轮半径 [m]
    WheelLoop<MecanumDrive> loop(lcfg,
        WheelSet<MecanumDrive>(wcfg, chassis, read_encoder, write_effort));

    loop.set_sink(on_sample, &loop);            // 可选

    // ---- 主循环：一拍就一行 ----
    const float dt = 0.001f;                    // 实测周期（不许标称值）
    loop.set_cmd(Twist{0.3f, 0.0f, 0.0f});      // 上游要求 (vx, vy, wz)
    for (uint32_t now = 0; now < 2000; ++now) {
        loop.tick(dt, now);                     // 限幅→目标→执行→测量→正解→里程计
        sim_step(dt);                           // 真机上没有这一步
        const Pose p = loop.pose();
        (void)loop.yaw_odo(); (void)loop.cmd(); (void)loop.twist();
        (void)loop.wheel_target(0); (void)loop.wheel_speed(0); (void)loop.wheel_effort(0);
        (void)p;
        // 姿态：跨库只传标量 —— est.observe_heading(loop.yaw_odo(), trust);
    }
    return 0;
}
```

## 4. 主循环的三条硬规矩

| 规矩 | 为什么 |
|---|---|
| `dt` 用**实测周期**，不许写标称值 | 标称 1 kHz 实际 980 Hz 会静默吃掉 2% 的位移；PID 的微分项也会抖 |
| `now` 由**调用方**给（ms 绝对时间基） | 库里不调 `millis()` —— 换平台不用改库；记录里的时间戳靠它 |
| **一拍只调一次 `tick()`** | 它是唯一心跳：限幅、轮控、测量、正解、里程计全在这一拍里 |

`sleep` / 分配 / 阻塞 I/O **不要**放进 `tick()` 之前的那段（MCU 上尤其）。

## 5. 数据往哪去

### 5.1 每拍一条记录（可选，但强烈建议）

`loop.set_sink(sink, ctx)` 转发给里程计，`sink` 的签名是 `void(*)(void* ctx, const lunokhod::odometry::OdometrySample&)`：

| 字段 | 是什么 | 用途 |
|---|---|---|
| `tick_` / `dt_` | 时间基 + 实测周期 | 记录/回放的骨架 |
| `ws_` | **实测轮速**（`WheelSpeeds`，含 `count_`） | L0 残差监测的原料 |
| `cmd_` | **限幅后**真正下发的车体指令 | 与 `twist_` 对比 = 残差 |
| `twist_` | 正解反推的车体速度 | 同上 |
| `pose_` | 同一拍的位姿 | 上位机显示 |

### 5.2 单轮只读口（诊断 / 显示用）

| 接口 | 是什么 | 边界 |
|---|---|---|
| `wheel_target(i)` | 该轮**目标**转速 | `i >= count_` 无意义 |
| `wheel_speed(i)` | 该轮**实测**转速（上一拍 `tick()` 的快照） | 未用到的轮恒为 0 |
| `wheel_effort(i)` | 该轮**最近一次算出的 effort**（`int16_t`） | 未用到的轮恒为 0；**不是实测**，是"我们让它出多大力" |

三个口都是**上一拍 `tick()` 的快照**（不是"现在去读一次编码器"）—— 所以在 `tick()` 之后取，数值自洽。

### 5.3 姿态融合怎么接

**跨库只传标量** —— 把连续角交出去，由调用方决定信多少：

```cpp
float yaw = loop.yaw_odo();          // 连续角，不 wrap（融合不许吃 wrap 后的角）
est.observe_heading(yaw, trust);     // foucault 的接口；trust 由你（调用方）给
```

**禁区**：让 lunokhod 的任何库 `#include` 姿态库的头，或反过来。

## 6. 五个常见坑（都踩过）

| 坑 | 症状 | 正解 |
|---|---|---|
| 自己 `for` 6 个轮子写 PWM | 未用到的轮子被写 0，驱动报故障或抖动 | 别碰；`apply()` 只动前 `count_` 个 |
| `wheel_speed(i)` / `wheel_effort(i)` 越界读 | 拿到 0，以为"轮子停了" | `i >= count_` 的槽**恒为 0**（表示"没有"）；按 `wheel_target(i)` 注释的边界用 |
| 单位混用（计数 / rpm / rad/s） | 位姿差一个常数倍，且**看起来"能跑"** | 只在 `read_encoder` 里做一次换算，之后全是 rad/s |
| 符号不一致 | 前进时位姿往回走 | 定死"正 = 车体前进"，用它校验四个轮子 |
| 消费时漏编上游算法库 | 链接期 `undefined reference to ctl::PID::...` | `third_party/ctlkit/src` 必须一起编（或用 CMake target 链，见下） |

## 7. 把它引进你的工程

三种用法（单库直编 / `add_subdirectory` / 库模式消费）见 [`../README.md`](../README.md) 「三种用法」。

CMake 最短路径 —— 链接 `chassis_loop` 就够了（它自己把其余全部带进来）：

```cmake
add_subdirectory(<lunokhod> ${CMAKE_BINARY_DIR}/lunokhod)
target_link_libraries(your_firmware PRIVATE chassis_loop)
```

## 8. 实测输出（就是 §3 那段代码）

```
tick 0  cmd.vx=0.002  twist.vx=0.000  pose=(0.000, 0.000)  wheel0=0.00
tick 400  cmd.vx=0.300  twist.vx=0.023  pose=(0.005, 0.000)  wheel0=0.76
tick 800  cmd.vx=0.300  twist.vx=0.024  pose=(0.015, 0.000)  wheel0=0.80
tick 1200  cmd.vx=0.300  twist.vx=0.025  pose=(0.025, 0.000)  wheel0=0.84
tick 1600  cmd.vx=0.300  twist.vx=0.026  pose=(0.035, 0.000)  wheel0=0.88
```

（PC 替身只求"能跑"，数值不追求物理真实：限幅在第一段爬升 —— `0.152 ≈ 1.5 m/s² × 0.1 s`，
之后 `cmd.vx` 停在 0.300；位姿随实测轮速累积。）
