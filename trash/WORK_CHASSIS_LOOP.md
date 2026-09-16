---
class: work
generated: false
accepted: true
---
> **类：D 施工单（临时物）** —— 参考实现，**不是代码源**。
> 唯一的代码源是 `inc/chassis_loop.hpp`（**用户手写**的那份）。**已验收（2026-09-16）→ 本文件已归档。**
> **设计权威：[`DESIGN.md`](DESIGN.md)** —— 契约 / 决策 D1~D9 / 职责边界 / 执行顺序 / 验收锚点
> 都在那边，**以它为准**；本文件只放「照着敲的那一份代码」和「怎么验收」。
> **分工**：`inc/` 你手写；`test/` AI 写（见 [`../../AGENTS.md`](../../AGENTS.md) §2）。

# WORK_CHASSIS_LOOP.md — 装配层施工单

## 1. 参考实现（AI 出，**已真跑过**）

> 交付方式照 `../../trash/WORK_ODOMETRY.md` 的先例：**AI 出参考实现 + 测试，你手敲 `inc/`**。
> 已在临时工程里编译 + 运行验证（`-Wall -Wextra -Werror` **零告警**，7 组测试全绿），
> 并按 §4 做了 12 个变异审计。下面是**逐字**的参考实现。

```cpp
#pragma once

// ============================================================================
// chassis_loop —— 装配层：把四块积木接到【同一个时间基】上，按固定顺序跑一遍。
//
// 它不生产数据，只搬运和排列。所有算法都在被它调用的库里。
//
//   限幅 → 逆解 → N×Wheel → 正解 → 里程计 → pose / yaw_odo
//
// 范围判据：**只吃「已算好的量」（Twist / dt / now），不碰 IO。**
// 一旦它需要知道「命令从哪来」「IMU 在哪读」→ 那不是装配层，是应用层。
// ============================================================================

#include "chassis.hpp"            // kinematics：底盘类型（模板参数）
#include "contracts.hpp"          // Twist / WheelSpeeds / Pose
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"
#include "wheel.hpp"

#include <cstdint>

// ── 参数聚合（规矩：常量与参数不许散落在函数体里）──────────────────────────
// 注意：这里【没有】底盘几何 —— 几何属于 Chassis，不属于装配层（见 D6）。
// 也【没有】control_dt_ —— 时间基唯一来源是 tick(dt, now)（见 D4）。
struct ChassisLoopConfig {
    // ── Twist 空间加速度限幅 ──
    float acc_vx_ = 1.5f;            // m/s²
    float acc_vy_ = 1.5f;
    float acc_wz_ = 4.0f;            // rad/s²

    // ── 轮控 ──
    PIDConfig           pid_{};
    SmoothPlannerConfig planner_{};

    // ── 里程计 ──
    float twist_scale_ = 1.0f;       // 标定系数（硬件属性）
};

template <typename Chassis>
class ChassisLoop {
public:
    // 与 Wheel 的 MeasureSpeedFn / SetPwmFn 同型
    using MeasureSpeedFn = float (*)(uint8_t wheel_id, float dt);
    using SetPwmFn       = void  (*)(uint8_t wheel_id, int16_t pwm);

    // 底盘【从外面注入】—— 几何是底盘自己的事（D6）。
    ChassisLoop(const ChassisLoopConfig& cfg,
                const Chassis&          chassis,
                MeasureSpeedFn          measure_speed,
                SetPwmFn                set_pwm)
        : chassis_(chassis),
          limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_),
          odom_(cfg.twist_scale_),
          w0_(0, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          w1_(1, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          w2_(2, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          w3_(3, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
          wheels_{&w0_, &w1_, &w2_, &w3_} {}

    // ── 上游接口 ──
    void set_cmd(const Twist& cmd) { cmd_in_ = cmd; }

    // ── 里程计记录出口（转发给 odometry 的 SampleSink）──
    // P19 的时间戳对齐原料 —— 见施工单 §6 的只读口表（D8）
    void set_sink(SampleSink sink, void* ctx = nullptr) {
        odom_.set_sink(sink, ctx);
    }

    // ── 唯一的心跳 ──
    //   dt  = 实测周期（不许用标称值；装配层不发明时间基）
    //   now = 绝对时间基（ms），由调用方提供 —— 它才是 IO 的拥有者
    void tick(float dt, uint32_t now) {
        // ① 限幅：先把上游野值收进物理可行范围，后面所有环节才安全
        cmd_out_ = limiter_.limit(cmd_in_, dt).out_;

        // ② 逆解：车体 Twist → N 轮目标
        target_ws_ = chassis_.inverse_kinematics(cmd_out_);

        const uint8_t n = target_ws_.count_;   // 底盘说几轮就几轮（diff=2 / omni=3 / mec=4）
        // 逆解只填前 n 个，尾巴是 indeterminate → 堵成 0，不许脏数据外流
        for (uint8_t i = n; i < 6; ++i) { target_ws_.values_[i] = 0.0f; }

        // ③ 下发 + 执行：每轮自己跑 S 曲线与 PID，出 PWM
        //    只碰实际用到的前 n 个；没被用到的 Wheel 永远不 update()
        //    （→ 不会写 PWM、不会调 MeasureSpeedFn）
        for (uint8_t i = 0; i < n; ++i) {
            wheels_[i]->set_cmd(target_ws_.values_[i]);
            wheels_[i]->update(dt);            // 内部【已经】读了编码器
        }

        // ④ 测量：取的是 ③ 同一拍读到的值。零初始化 —— 未用到的槽留 0
        WheelSpeeds meas{};
        meas.count_ = n;
        for (uint8_t i = 0; i < n; ++i) {
            meas.values_[i] = wheels_[i]->get_speed();
        }

        // ⑤ 正解：N 轮实测 → 车体 Twist
        twist_meas_ = chassis_.forward_kinematics(meas);

        // ⑥ 里程计：积分成位姿。cmd = 本拍【真正下发】的那一份（限幅后），
        //    twist = 本拍【实测反推】的那一份 —— 这两笔是 L0 残差的原料
        odom_.update(twist_meas_, dt, now, cmd_out_, &meas);
    }

    // ── 只读口（P19 的数据入口，不是调试附属品）────────────────────────
    Pose  pose() const { return odom_.pose(); }        // 位姿（yaw 已 wrap）
    float yaw_odo() const { return odom_.yaw_ref(); }  // 连续角 —— 融合钩子吃这个
    Twist twist() const { return twist_meas_; }        // ⑤ 正解输出（**未**标定）
    Twist cmd() const { return cmd_out_; }             // ① 限幅后、真正下发的
    float wheel_speed(uint8_t i) const { return wheels_[i]->get_speed(); }
    float wheel_target(uint8_t i) const { return target_ws_.values_[i]; }

private:
    Chassis          chassis_;
    TwistAccLimiter  limiter_;
    Odometry         odom_;

    Wheel  w0_, w1_, w2_, w3_;
    Wheel* wheels_[4];

    Twist       cmd_in_{0.0f, 0.0f, 0.0f};   // 上游要求（限幅前）
    Twist       cmd_out_{0.0f, 0.0f, 0.0f};  // 限幅后、真正下发
    WheelSpeeds target_ws_{};                // 逆解出来的 N 轮目标
    Twist       twist_meas_{0.0f, 0.0f, 0.0f};  // ⑤ 正解输出
};
```

## 2. `CMakeLists.txt`（参考）

```cmake
cmake_minimum_required(VERSION 3.28)
project(lunokhod_chassis_loop CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖五个库：优先用消费方已引入的 target，没有才回退本地相对路径
# （理由与写法见 ../../odometry/CMakeLists.txt —— 无条件 add_subdirectory 会 target 重名）
if(NOT TARGET contracts)
    add_subdirectory(../contracts ${CMAKE_BINARY_DIR}/_ext/contracts)
endif()
foreach(dep kinematics odometry)
    if(NOT TARGET ${dep})
        add_subdirectory(../${dep} ${CMAKE_BINARY_DIR}/_ext/${dep})
    endif()
endforeach()
foreach(dep wheel twist_acc_limiter)
    if(NOT TARGET ${dep})
        add_subdirectory(../control/${dep} ${CMAKE_BINARY_DIR}/_ext/${dep})
    endif()
endforeach()

# 库本体：**INTERFACE** —— 全模板 + POD 配置，没有 .cpp（DESIGN.md §9 F2）
add_library(chassis_loop INTERFACE)
target_include_directories(chassis_loop INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(chassis_loop INTERFACE
    contracts kinematics odometry wheel twist_acc_limiter)

# 两种构建模式守卫（AGENTS.md §5.1：被引入时只出库）
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    return()
endif()

enable_testing()
add_executable(test_chassis_loop test/test_chassis_loop.cpp)
target_link_libraries(test_chassis_loop PRIVATE chassis_loop)
target_compile_options(test_chassis_loop PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_chassis_loop COMMAND test_chassis_loop)
```

**还要在根 `CMakeLists.txt` 的依赖序末尾加一行** `add_subdirectory(chassis_loop)`
（它依赖其余全部，所以必须放最后）。

## 3. 测试（已写在 [`test/test_chassis_loop.cpp`](../test/test_chassis_loop.cpp)）

**oracle 一览（每个期望值都能回答「它从哪来」）**：

| 组 | 测什么 | oracle |
|---|---|---|
| 1 | 逆解手算锚点 | **手算**：`vx/r = 0.3/0.03 = 10`；`(lx+ly)/r·wz = 0.22/0.03 × 0.3 = 2.2`；混合 `10 ∓ 2.2` |
| 2 | 限幅的性质 + 到位拍数 | **手算**：`ceil(1.0 / (1.5 × 0.01)) = 67` 拍；**性质**：每拍增量 ≤ `acc·dt`、不超调 |
| 3 | `cmd()` = 限幅后 **且** 限幅在逆解前 | **手算**：`1.5 × 0.01 = 0.015`；`0.015/0.03 = 0.5 rad/s` |
| 4 | 执行顺序 + 未用到的轮不被触碰 | **记录**：假 IO 记下「谁在何时被调」；`DiffDrive → count_ = 2` |
| 5 | `dt` / `now` 穿透（抓「自己发明时间基」） | **穿透**：`dt = 0.013`、`now = 1234567` 必须原样到达（含故意回拨 `now`） |
| 6 | 互逆 `实测 = 目标 → twist() 回到 cmd()` | **性质**：互逆 + 装配层不额外改数 |
| 7 | 逆解结果**真的下发到轮子** | **手算**：planner 直通（`Tf_ = 0` → alpha = 1）+ `kp = 1` → `PWM = 目标` |

**容差推导**（不许手拍）：float 相对 eps = 1.19e-7，本例算术最多 3 次乘除 → 累积 ≈ 3.6e-7，
量级最大 10.0 → 绝对 3.6e-6，取 4 倍安全系数 → **2e-5**。
第 7 组的 `±1` 是 **int16 截断**的界，不是浮点容差。

## 4. 判别力审计（12 个变异，**全部变红**）

> 做法：把参考实现的**关键行逐个改坏**，重建并跑测试。若某个变异「测试仍然全绿」，
> 说明那条断言是摆设。

| 变异 | 改坏成 | 变红的断言 |
|---|---|---|
| M1 | `cmd()` 返回限幅**前**的值 | 8 条 |
| M2 | 限幅挪到逆解**之后** | 39 条 |
| M3 | 轮控用标称 `dt = 0.001` | 1 条 |
| M4 | 里程计用内部计数器当时间基 | 2 条 |
| M4b | 里程计收到标称 `dt` | 2 条 |
| M5 | 无视 `count_`，永远跑 4 轮 | 7 条 |
| M6 | 正解吃**目标**而不是**实测** | 1 条 |
| M7 | 不堵 `WheelSpeeds` 的尾巴（DESIGN.md §9 F1） | 2 条 |
| M8 | 忘了 `set_cmd` 下发目标 | 4 条 |
| M9 | 所有轮子都下发第 0 个槽 | 3 条 |
| M11 | `twist()` 直接返回命令（假残差） | 1 条 |
| M12 | `set_sink` 不转发 | 7 条 |
| M13 | 记录里的 `count_` 写错 | 1 条 |

**过程中测试抓到 AI 自己两个错**（这才是尺子该有的样子）：
① 第 7 组的手算期望值漏乘了 `wz = 0.3`（写成 `22/3`，实际应是 `2.2`）；
② 第 7 组最初用 `kp = 9` 配两级 LPF 的 `1/9` 增益凑整，机理绕且易错 ——
改成 `Tf_ = 0`（alpha = dt/(Tf+dt) = 1，planner 直通）后推导只剩一步。

## 5. 落地顺序

```
① ⬜ 你手写 chassis_loop/inc/chassis_loop.hpp   ← 照 §1（逐字）；语义疑问查 DESIGN.md
② ⬜ 你写 chassis_loop/CMakeLists.txt           ← 照 §2
   ⚠ 不需要 src/chassis_loop.cpp（INTERFACE 库，见 DESIGN.md §9 F2）
③    根 CMakeLists.txt 末尾加 add_subdirectory(chassis_loop)
④    cmake -S . -B build && ctest --test-dir build --output-on-failure    → 9/9
⑤    scripts/check_docs.py && scripts/ci_local.py
      同时**补组件清单**：README.md 的库清单 / AGENTS.md §5 依赖图 / docs/ARCHITECTURE.md §2
      （组件真落地后才加 —— 现在加就是谎报仓库里有）
⑥    改 ~/Develop/Workspace/KND_Trial/firmware/ 的 app.cpp 用本组件
      → 仿真输出**逐位比对**（锚点见 DESIGN.md §11）
⑦    本文件进 ../../trash/
```

### 5.1 手敲时盯住这三条（写错就变红）

1. `cmd_out_ = limiter_.limit(cmd_in_, dt).out_;` —— 逆解吃 `cmd_out_`，**不是** `cmd_in_`
2. `for (uint8_t i = n; i < 6; ++i) { target_ws_.values_[i] = 0.0f; }` —— 堵尾巴那两行
3. `odom_.update(twist_meas_, dt, now, cmd_out_, &meas);` —— 四个参数各是各的

## 6. 验收后的处置（D 类规则）

- 本文件进 `../../trash/`（连同 `../../trash/README.md` 的待裁决清单）。
- ⚠ **移走前先确认 `DESIGN.md` 已覆盖本文件里的所有【事实】**（契约 / 决策 / 边界），
  否则事实会跟着一个「临时物」一起消失 —— 那正是本节存在的意义。
- 处置后：`AGENTS.md` §5 依赖图、`README.md` 库清单、`docs/ARCHITECTURE.md` §2 要补上 `chassis_loop`。
