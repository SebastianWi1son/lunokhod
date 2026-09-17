---
class: work
generated: false
accepted: true
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

> ## ✅ 本施工单已落地（2026-09-17）
> 下面 §3 的两个代码块 = **仓库里 `inc/` 的实际内容**（逐字同步，随时可对照）。
> **契约以 [`DESIGN.md`](DESIGN.md) §5.2 为准，代码以 `inc/` 为准**；本文件只剩归档价值。

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

## 3.0 相比你**当前那份**，改哪几处（逐处对照）

> 规则：本文档**不写行号**（文档体系规矩）—— 用「符号名 + 待搜索的旧文本」定位。
> **目标版全文**见 §3.2；`wheel_set.hpp` 是**全新文件**，无旧版本可对照。
> ✅ 标记的表示**你那份已经对了，不要动**。

| # | 位置（符号） | 你现在的 | 改成 | 动作 |
|---|---|---|---|---|
| 1 | 文件头注释 | 无 | 加一句「它**不认识**底盘与轮子 —— 中间那一格是执行器组」 | 加（可选） |
| 2 | `#include` 区 | `#include "chassis.hpp"` 与 `#include "wheel.hpp"` | **删掉这两行** | 删 |
| 3 | `ChassisLoopConfig` | 只有 `acc_*` + `twist_scale_` | 同左 | ✅ 不动 |
| 4 | 类模板参数 | `template <typename ActuatorSet>` | 同左 | ✅ 不动 |
| 5 | **构造函数初始化列表** | `: actuators_() {}` | `: actuators_(actuators), limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_), odom_(cfg.twist_scale_) {}` | **改（真 bug）** |
| 6 | `tick()` 的 ②~⑤ 步 | 逆解 + 堵尾巴 + 轮子循环 + 组装 `meas` + 正解（约 18 行） | 4 行：`inverse` → `apply` → `measure` → `forward` | **改（核心）** |
| 7 | `tick()` 的里程计那行 | `odom_.update(..., &meas);` + 上一行残留注释 `// --- process to res残差 ---` | `odom_.update(..., &meas_);`，残留注释**删掉** | 改 |
| 8 | 只读口 `wheel_speed(i)` | `wheels_[i]->get_speed()` | `meas_.values_[i]` | 改 |
| 9 | 只读口 `wheel_target(i)` | `ws_target_.values_[i]` | 同左 | ✅ 不动 |
| 10 | 私有成员区 | `Chassis chassis_;` ＋ `Wheel w0_…w5_;` ＋ `Wheel* wheels_[6];` | **三者全删**；加 `ActuatorSet actuators_;`（放最前，初始化顺序要对）与 `WheelSpeeds meas_{};` | **改** |

**三件容易误判的事**

1. **第 5 条是真 bug**：`actuators_()` 把注入进来的执行器组**丢掉**了（默认构造一个），
   而且 `limiter_` / `odom_` 连初始化都没有 → `ChassisLoopConfig` 里的参数**根本没用上**。
   正确写法是 `actuators_(actuators)` + 另外两行。
2. **第 6 条里"堵尾巴"那两行不是丢了** —— 它搬进了 `WheelSet::inverse()`
   （谁造 `WheelSpeeds` 谁负责堵，见 §3.1）。`meas` 也从**局部变量**升级为**成员** `meas_`：
   里程计要它、只读口也要它。
3. **`chassis_` 不是被删掉，是搬进 `WheelSet` 了** —— 装配层不再持有底盘，
   但它没消失（`WheelSet` 持有它，并用它做正/逆解）。

## 3. 参考实现（AI 出，**已真跑过**）

验证（**两遍**：一套用 AI 的成员名、一套用**你的**成员名 `t_cmd_in_` / `t_cmd_final_` / `ws_target_`）：
9 组测试 `ALL PASS` · 严格档（`-Wconversion -Wshadow -pedantic`）**零告警** ·
ASan+UBSan 干净 · **KND_Trial 仿真输出逐位相同**（md5 `9ab64f43…`）。

### 3.1 `inc/wheel_set.hpp`（✅ 已落地）

```cpp
#pragma once

#include "chassis.hpp"
#include "chassis_loop.hpp"
#include "contracts.hpp"
#include "wheel.hpp"
#include <array>
#include <cstdint>

struct WheelSetConfig {
    PIDConfig pid_{};
    SmoothPlannerConfig planner_{};
};

template <typename Chassis>
class WheelSet {
public:
    // --- constructor ---
    WheelSet(const WheelSetConfig& cfg, const Chassis& chassis, MeasureSpeedFn measure_speed, SetEffortFn set_effort)
        : chassis_(chassis),
          wheels_{
               {Wheel(0, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  Wheel(1, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  Wheel(2, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  Wheel(3, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  Wheel(4, cfg.planner_, cfg.pid_, measure_speed, set_effort),
                  Wheel(5, cfg.planner_, cfg.pid_, measure_speed, set_effort)}
          } {}

    // --- twist -> wheelspeeds ---
    WheelSpeeds inverse(const Twist& twist) const {
        WheelSpeeds sp = chassis_.inverse_kinematics(twist);
        for (uint8_t i = sp.count_; i < 6; ++i) { sp.values_[i] = 0; }
        return sp;
    }

    // --- push down + execute ---
    void apply(const WheelSpeeds& sp, float dt) {
        const uint8_t wn = sp.count_;
        for (uint8_t i = 0; i < wn; ++i) {
            wheels_[i].set_cmd(sp.values_[i]);
            wheels_[i].update(dt);
        }
    }

    // --- get measure ----
    // 活跃数取自传入的设定值（与 apply 对称）→ 本类除轮子外无状态
    WheelSpeeds measure(const WheelSpeeds& sp) const {
        WheelSpeeds fb{};
        fb.count_ = sp.count_;
        for (uint8_t i = 0; i < sp.count_; ++i) { fb.values_[i] = wheels_[i].get_speed(); }
        return fb;
    }

    // --- wheelspeeds -> twist ---
    Twist forward(const WheelSpeeds& fb) const { return chassis_.forward_kinematics(fb); }

private:
    Chassis chassis_;
    std::array<Wheel, 6> wheels_;               ///< room for wheels
};

template <typename Chassis>
using WheelLoop = ChassisLoop<WheelSet<Chassis>>;
```

### 3.2 `inc/chassis_loop.hpp`（✅ 已落地）

```cpp
#pragma once

#include "contracts.hpp"
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"
#include <cstdint>

struct ChassisLoopConfig {
    // --- Twist Acc limiter ---
    float acc_vx_ = 1.5f;
    float acc_vy_ = 1.5f;
    float acc_wz_ = 4.0f;
    // --- odometry ---
    float twist_scale_ = 1.0f;
};

template <typename ActuatorSet>
class ChassisLoop {
public:
    ChassisLoop(const ChassisLoopConfig& cfg, const ActuatorSet& actuators)
        : actuators_(actuators), limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_), odom_(cfg.twist_scale_) {}

    // ----- upstream interface -----
    void set_cmd(const Twist& cmd) { t_cmd_in_ = cmd; }
    // ----- odom record sink -----
    void set_sink(SampleSink sink, void* ctx = nullptr) { odom_.set_sink(sink, ctx); }
    // ----- heartbeat tick -----
    void tick(float dt, uint32_t now) {
        // --- limiter ---
        t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_;
        // --- inverse to ws ---
        ws_target_ = actuators_.inverse(t_cmd_final_);
        // --- push down + execute ---
        actuators_.apply(ws_target_, dt);
        // --- measure ---
        ws_meas_ = actuators_.measure(ws_target_);
        // --- forward to twist ---
        twist_meas_  = actuators_.forward(ws_meas_);
        // --- odom integral to pose ---
        // --- process to res残差 ---
        odom_.update(twist_meas_, dt, now, t_cmd_final_, &ws_meas_);
    }

    // ----- getters -----
    Pose pose() const { return odom_.pose(); }
    float yaw_odo() const { return odom_.yaw_ref(); }
    Twist twist() const { return twist_meas_; }
    Twist cmd() const { return t_cmd_final_; }
    float wheel_speed(uint8_t i) const { return ws_meas_.values_[i]; }
    float wheel_target(uint8_t i) const { return ws_target_.values_[i]; }

private:
    ActuatorSet actuators_;
    TwistAccLimiter limiter_;
    Odometry odom_;

    Twist t_cmd_in_{ 0.0f, 0.0f, 0.0f };            ///< upstream cmd
    Twist t_cmd_final_{ 0.0f, 0.0f, 0.0f };         ///< actual twist push down
    WheelSpeeds ws_target_{};                                    ///< inverse result target
    WheelSpeeds ws_meas_{};
    Twist twist_meas_{ 0.0f, 0.0f, 0.0f };          ///< forward output
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

## 5. 落地顺序（2026-09-17 更新：测试已先行切好）

```
✅ 前置：测试已切到接缝版（`test/test_chassis_loop.cpp`），**8 组断言一个字没改**，
        新增第 9 组「接缝一致性」（用假执行器组 StubSet 证明接缝可替换）。
        → 在头文件落地前，该测试目标编译不过 —— **预期状态**（文件顶部有横幅说明）。
① ⬜ 你手写 inc/wheel_set.hpp + 重写 inc/chassis_loop.hpp（照 §3 逐字）
       可选捎带 TODO **P25**（暴露限幅饱和标志，3 行，同一个文件）
②    说一声 → AI 跑：严格档 + Release + ASan/UBSan + 聚合 ctest + 消费模式 + 变异审计
③    ⏸ **按用户决定暂缓**：KND_Trial 是库外工程，本轮不碰（它当前也因 ctlkit 迁移编不过，
        见 `../../docs/TODO.md` P30）。锚点已在 /tmp 隔离副本上验过逐位相同。
④    本文件进 ../../trash/

**已验证过一遍（/tmp 隔离验证，用当前上游 ctlkit v0.1.1）**：
严格档零告警 · ASan/UBSan 干净 · 9 组 ALL PASS · **KND 仿真输出逐位相同**（md5 `9ab64f43…`）。

> **承接上游（2026-09-17）**：算法原语已 vendor 到 `third_party/ctlkit` v0.1.1，
> `PIDConfig` 变成 `gains_/limits_/tunings_` 三段 + 具名链式设置器。
> 接缝的设计**不受影响**（`WheelSetConfig` 照样只放一个 `PIDConfig`）；
> ⚠ **外仓消费方 KND_Trial 的 `app.hpp` 用旧扁平字段、当前编译不过** ——
> **本轮按用户决定不碰下游**（修法很小：那 10 行换成一处具名链式调用；见 `../../docs/TODO.md` P30）。

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
