---
class: work
generated: false
accepted: false
---
> **类：D 施工单（临时物）** —— "通用框架"（执行器组接缝）的参考实现，**不是代码源**。
> 唯一的代码源是 ⬜ `inc/wheel_set.hpp` + ⬜ `inc/chassis_loop.hpp`（**你手写**的那份）。验收后进 `../../trash/`。
> **设计权威**：[`DESIGN.md`](DESIGN.md) §5.2（接缝契约）+ §8.2 **D11**（决策）——
> "先改设计再改代码"这条**已经做了**（设计已更新）。
> **分工**：`inc/` 你手写；`test/` AI 写（验收时我换调用点 + 加第 9 组接缝测试）。

# WORK_SEAM.md — 执行器组接缝（"通用框架"）

## 0. 一句话

把装配层中间那一格「N×Wheel」抽成**一个可替换的积木（执行器组）**。
装配层从此**不认识** `Chassis` / `Wheel` —— 换执行方式不必改编排。

## 1. 为什么做（两个真实压力 + 一个不用管）

| 压力 | 它变的是什么 | 接缝吃不吃得下 |
|---|---|---|
| **FOC**（若它自带速度环，对外只收"速度模式"） | 换掉"轮控"这一块 | ✅ 换一个 `ActuatorSet` |
| **Swerve** | 1 模块 = 2 执行器（转向 + 驱动）+ 角度反馈 | ✅ 类型层面已装得下；**输出契约**的扩展留 [`../../docs/TODO.md`](../../docs/TODO.md) **P26** |
| **普通 FOC**（电流环跑在驱动里） | 只是 HAL 之下换个实现 | ❌ 不用动 —— `±1000 effort` 语义已经留好 |

## 2. 改动范围（三处，别多改）

1. **新增** ⬜ `inc/wheel_set.hpp` —— 执行器组（含 `WheelLoop` 便捷别名）
2. **重写** ⬜ `inc/chassis_loop.hpp` —— 编排层：只认执行器组契约
3. **调用点**：`ChassisLoop<MecanumDrive>(cfg, MecanumDrive(...), io)`
   → `WheelLoop<MecanumDrive>(cfg, WheelSet<MecanumDrive>(wcfg, MecanumDrive(...), io))`

**公开只读口一个都不变**（`pose` / `yaw_odo` / `twist` / `cmd` / `wheel_speed` / `wheel_target`）
→ 使用方只改构造那两行。

## 3. 参考实现（AI 出，**已真跑过**）

验证：9 组测试 `ALL PASS` · 严格档（`-Wconversion -Wshadow -pedantic`）**零告警** ·
ASan+UBSan 干净 · **KND_Trial 仿真输出逐位相同**（md5 `9ab64f43…`）。

### 3.1 ⬜ `inc/wheel_set.hpp`

```cpp
#pragma once

// ============================================================================
// WheelSet —— 「执行器组」的一个实现：把「车体 Twist ↔ N 个轮子」这一格包起来。
//
// 它是 ChassisLoop 的**接缝**（见 docs/DESIGN.md §5.2）：装配层不认识 Chassis、
// 也不认识 Wheel，只认这四件事：
//
//   WheelSpeeds inverse(const Twist&) const;   // 车体 → 每轮目标（未用到的槽堵 0）
//   void        apply(const WheelSpeeds&, float dt);  // 下发 + 执行（只动前 count_ 个）
//   WheelSpeeds measure() const;               // 读回实测（count_ = 活跃数）
//   Twist       forward(const WheelSpeeds&) const;    // 实测 → 车体
//
// 将来 FOC（自带走环）或 Swerve（每模块两个执行器）各写一个同类插进来即可 ——
// 见 docs/TODO.md P25 / P26。**在那里之前不细化这个契约**。
// ============================================================================

#include "chassis.hpp"            // kinematics：底盘类型（模板参数）
#include "chassis_loop.hpp"       // ChassisLoop（末尾给它一个便捷别名）
#include "contracts.hpp"          // Twist / WheelSpeeds
#include "wheel.hpp"              // Wheel / PIDConfig / SmoothPlannerConfig

#include <array>
#include <cstdint>

// 执行器组自己的参数 —— 编排层不该知道 PID（那是执行器的事）
struct WheelSetConfig {
    PIDConfig           pid_{};
    SmoothPlannerConfig planner_{};
};

template <typename Chassis>
class WheelSet {
public:
    WheelSet(const WheelSetConfig& cfg,
             const Chassis&          chassis,
             MeasureSpeedFn          measure_speed,
             SetPwmFn                set_pwm)
        : chassis_(chassis),
          // ⚠ 必须是 std::array 实打实的对象，**不存指针**：
          //   存 `Wheel* wheels_[6]` 的话，这个类一旦被【按值传递】（注入就是这样），
          //   拷贝出来的指针仍指向【原对象】的 Wheel → 临时对象一死就悬空 → 段错误。
          wheels_{{ Wheel(0, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
                    Wheel(1, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
                    Wheel(2, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
                    Wheel(3, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
                    Wheel(4, cfg.planner_, cfg.pid_, measure_speed, set_pwm),
                    Wheel(5, cfg.planner_, cfg.pid_, measure_speed, set_pwm) }} {}

    // 车体 → 每轮目标。容量 = 契约容量 6；活跃数由底盘说（DESIGN.md §8.2 D10）
    WheelSpeeds inverse(const Twist& twist) const {
        WheelSpeeds sp = chassis_.inverse_kinematics(twist);
        // 逆解只填前 count_ 个，尾巴是 indeterminate → 堵成 0（DESIGN.md §9 F1）
        for (uint8_t i = sp.count_; i < 6; ++i) { sp.values_[i] = 0.0f; }
        return sp;
    }

    // 下发 + 执行：只碰实际用到的前 count_ 个（没被用到的轮子永不 update）
    void apply(const WheelSpeeds& sp, float dt) {
        const uint8_t n = sp.count_;
        for (uint8_t i = 0; i < n; ++i) {
            wheels_[i].set_cmd(sp.values_[i]);
            wheels_[i].update(dt);
        }
        count_ = n;
    }

    // 读回实测。零初始化 —— 未用到的槽留 0（不许脏数据外流）
    WheelSpeeds measure() const {
        WheelSpeeds fb{};
        fb.count_ = count_;
        for (uint8_t i = 0; i < count_; ++i) { fb.values_[i] = wheels_[i].get_speed(); }
        return fb;
    }

    // 实测 → 车体
    Twist forward(const WheelSpeeds& fb) const { return chassis_.forward_kinematics(fb); }

private:
    Chassis                chassis_;
    std::array<Wheel, 6>   wheels_;      // 容量 = 契约容量（D10）；活跃数见 count_
    uint8_t                count_ = 0;   // 最近一次 apply 的活跃轮数（measure 用它）
};

// 常见组合的便捷别名：WheelLoop<MecanumDrive> = ChassisLoop<WheelSet<MecanumDrive>>
template <typename Chassis>
using WheelLoop = ChassisLoop<WheelSet<Chassis>>;
```

### 3.2 ⬜ `inc/chassis_loop.hpp`

```cpp
#pragma once

// ============================================================================
// chassis_loop —— 装配层：把积木接到【同一个时间基】上，按固定顺序跑一遍。
//
//   限幅 → 执行器组.inverse → 执行器组.apply → 执行器组.measure → 执行器组.forward → 里程计
//
// 它不生产数据，只搬运和排列。范围判据：**只吃「已算好的量」，不碰 IO。**
//
// 注意它**不认识** Chassis / Wheel —— 中间那一格是「执行器组」（ActuatorSet 模板参数），
// 契约见 docs/DESIGN.md §5.2。这样换执行方式（FOC / Swerve）不必改编排。
// ============================================================================

#include "contracts.hpp"          // Twist / WheelSpeeds / Pose
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"

#include <cstdint>

// 编排层自己的参数 —— 只有限幅与标定（PID / planner 在执行器组的配置里）
struct ChassisLoopConfig {
    float acc_vx_ = 1.5f;            // m/s²，Twist 空间加速度限幅
    float acc_vy_ = 1.5f;
    float acc_wz_ = 4.0f;            // rad/s²
    float twist_scale_ = 1.0f;       // 里程计标定系数（硬件属性）
};

template <typename ActuatorSet>
class ChassisLoop {
public:
    // 执行器组【从外面注入】—— 与底盘、IO 一样，都是注入
    ChassisLoop(const ChassisLoopConfig& cfg, const ActuatorSet& actuators)
        : actuators_(actuators),
          limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_),
          odom_(cfg.twist_scale_) {}

    // ── 上游接口 ──
    void set_cmd(const Twist& cmd) { cmd_in_ = cmd; }

    // ── 里程计记录出口（转发 odometry 的 SampleSink）—— P19 的时间戳原料 ──
    void set_sink(SampleSink sink, void* ctx = nullptr) { odom_.set_sink(sink, ctx); }

    // ── 唯一的心跳：dt = 实测周期；now = 绝对时间基（调用方给）──
    void tick(float dt, uint32_t now) {
        cmd_out_    = limiter_.limit(cmd_in_, dt).out_;        // ① 限幅
        target_ws_  = actuators_.inverse(cmd_out_);             // ② 车体 → 执行器目标
        actuators_.apply(target_ws_, dt);                       // ③ 下发 + 执行
        meas_       = actuators_.measure();                     // ④ 读回实测
        twist_meas_ = actuators_.forward(meas_);                // ⑤ 实测 → 车体
        odom_.update(twist_meas_, dt, now, cmd_out_, &meas_);   // ⑥ 里程计
    }

    // ── 只读口（P19 的数据入口，不是调试附属品）──
    Pose  pose() const { return odom_.pose(); }
    float yaw_odo() const { return odom_.yaw_ref(); }
    Twist twist() const { return twist_meas_; }        // ⑤ 正解输出（**未**标定）
    Twist cmd() const { return cmd_out_; }             // ①【限幅后】、真正下发的
    float wheel_speed(uint8_t i) const { return meas_.values_[i]; }
    float wheel_target(uint8_t i) const { return target_ws_.values_[i]; }

private:
    ActuatorSet     actuators_;
    TwistAccLimiter limiter_;
    Odometry        odom_;

    Twist       cmd_in_{0.0f, 0.0f, 0.0f};   // 上游要求（限幅前）
    Twist       cmd_out_{0.0f, 0.0f, 0.0f};  // 限幅后、真正下发
    WheelSpeeds target_ws_{};                // ② 执行器目标
    WheelSpeeds meas_{};                     // ④ 实测
    Twist       twist_meas_{0.0f, 0.0f, 0.0f};  // ⑤ 正解输出
};
```

## 4. 调用点怎么变（以 KND_Trial 的 `app.cpp` 为例）

```cpp
// 旧：装配层自己造底盘与轮子
ChassisLoopConfig c;
c.acc_vx_ = ...; c.pid_ = ...; c.planner_ = ...;      // 一份配置管到底
loop_(make_loop_cfg(cfg),
      MecanumDrive(lx, ly, r),
      hal::measure_speed, hal::set_pwm)

// 新：参数各归各家，执行器组被注入
ChassisLoopConfig lc;  lc.acc_vx_ = ...;              // 编排层：限幅 + 标定
WheelSetConfig    wc;  wc.pid_ = ...; wc.planner_ = ...;// 执行器组：PID + planner
loop_(make_loop_cfg(cfg),
      WheelSet<MecanumDrive>(make_wset_cfg(cfg),
                             MecanumDrive(lx, ly, r),
                             hal::measure_speed, hal::set_pwm))
```

## 5. 落地顺序

```
① ⬜ 你手写 inc/wheel_set.hpp + 重写 inc/chassis_loop.hpp（照 §3 逐字）
②    说一声 → AI 换测试调用点（**8 组断言一个字不改**）+ 加第 9 组「接缝一致性」测试
③    AI 跑：四档编译 + 聚合 9→10 · 消费模式 + 变异审计 + KND 逐位
④    AI 改 KND_Trial 的 app（配置拆两份 + 注入 WheelSet）→ 逐位比对
⑤    本文件进 ../../trash/
```

## 6. 一个必须记住的坑（AI 本轮踩了，已入账本）

**自引用指针成员 + 按值传递 = 悬空。**

```cpp
Wheel* wheels_[6];      // ✗ 拷贝后仍指向【原对象】的 Wheel
```

第一版参考实现写的是 `Wheel* wheels_[6]` + `wheels_{&w0_, …}`：
编译过、9 组测试也全过（因为测试是就地构造的），**但 KND 一跑就段错误** ——
执行器组是**按值注入**的，拷贝发生在注入那一刻，临时对象一死指针就悬空了。

**修法**：用 `std::array<Wheel, 6>` 存**实打实的对象**，不存指针（见 §3.1）。

## 6.1 与待办 P25 的关系（同一个文件，别漏）

`TODO.md` **P25**（暴露限幅饱和标志）也要改 `inc/chassis_loop.hpp`，**3 行**：
`tick()` 里存下整个 `LimitResult`（`lim_res_ = limiter_.limit(...)`）+ 一个 getter + 一个成员（带 `{}`）。
**与接缝版不冲突** —— 接缝版把那行调用挪了位置，但仍是同一次 `limit()` 调用。
**建议你敲的时候一次带上**（否则还要再开一遍那个文件）；测试我在第 ② 步一起补。

## 7. 手敲时盯住三条

1. `WheelSet` 里**不要出现 `Wheel*`** —— 用 `std::array<Wheel, 6>`
2. 堵尾巴那两行（`for (i = sp.count_; i < 6; ++i) ... = 0.0f`）在 **`WheelSet::inverse`**，
   不在装配层了（谁造 `WheelSpeeds` 谁负责堵）
3. `ChassisLoop` 里那六步的顺序与名字**不许动**（①限幅 ②inverse ③apply ④measure ⑤forward ⑥里程计）
