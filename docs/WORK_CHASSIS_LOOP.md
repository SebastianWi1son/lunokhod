---
class: work
generated: false
accepted: false
---
> **类：D 施工单（临时物）** —— 参考实现要点，**不是代码源**。
> 唯一的代码源是 ⬜ `chassis_loop/inc/chassis_loop.hpp`（**你手写**的那份）。
> 验收通过后本文件进 `trash/`。
>
> **设计权威**：[`ARCHITECTURE.md`](ARCHITECTURE.md) §2（下行链）·
> [`../kinematics/docs/THEORY.md`](../kinematics/docs/THEORY.md) §5.2（`ChassisController` 的历史提案）
> **动机**：[`TODO.md`](TODO.md) P20
> **分工**：`inc/` `src/` 你手写；`test/` AI 写（见 [`../AGENTS.md`](../AGENTS.md) §2）

# WORK_CHASSIS_LOOP.md — 装配层施工单

## 0. 一句话

**把 lunokhod 的四块积木接到同一个时间基上，按固定顺序跑一遍。**
它**不生产数据，只搬运和排列** —— 所有算法都在被它调用的库里。

## 1. 它解决什么问题（先看证据）

同一个装配链，现在手写了两遍：

| 位置 | 行数 |
|---|---|
| `~/Develop/Workspace/fw_poc/` | `main.cpp` 110 行 |
| `~/Develop/Workspace/KND_Trial/firmware/` | `app.cpp` 68 行 |

**两遍写的是同一条链。** 而链里有**易错知识**：

1. `tick` 的**顺序**（限幅必须在逆解前、里程计必须在正解后）
2. `dt` 必须用**实测值**，不许用标称值（喂标称 1 ms 而实际 1.2 ms = 系统性多算 20%）
3. 姿态融合钩子必须在**里程计之后**（它要吃本拍最新的 `yaw_ref`）
4. 参数必须聚合（`PIDConfig` / `SmoothPlannerConfig` 不能散落在函数体里）

**每重写一遍就有一次写错的机会。**

## 2. 范围（钉死）

```
set_cmd(Twist) → 限幅 → 逆解 → N×Wheel → 正解 → 里程计 → pose / yaw_ref
```

**只做下行链 + 里程计。**

### 判据（越界自查）

> **装配层只吃「已算好的量」（`Twist` / `dt` / `now`），不碰 IO。**
> 一旦它需要知道「命令从哪来」「IMU 在哪读」 → **那不是装配层，是应用层。**

### 不做的两件事（各有理由）

| 不做 | 为什么 |
|---|---|
| **姿态融合** | 碰了就要 `#include "estimator.hpp"` → **lunokhod 依赖 foucault** ✗（见 `../AGENTS.md` §5.3「跨库只传标量」）。融合由使用者接一根线：`ahrs.observe_heading(loop.yaw_ref())` |
| **修正输入**（`δcmd`） | `ARCHITECTURE.md` §3.3 自己写着「这一层在两个设计里都是空白」。**为没设计出来的东西预留接口 = 猜** |

## 3. 位置与依赖

```
lunokhod/
├── contracts/         ← 最底层
├── kinematics/        ← 逆解 / 正解
├── odometry/          ← 积分 + 记录
├── control/
│   ├── twist_acc_limiter/
│   └── wheel/
└── ⬜ chassis_loop/    ← 本组件（在 control/ 之外 —— 它编排的不只是控制）
```

依赖方向（**单向、无环**，本组件在**最上层**）：

```
contracts ← kinematics ←┐
contracts ← odometry   ←┼← chassis_loop ← （固件 / 应用）
contracts ← twist_acc_limiter ←┘
contracts ← wheel      ←┘
```

## 4. 对外接口（契约）

```cpp
// ⬜ chassis_loop.hpp —— 你手写

#include "chassis.hpp"            // kinematics：底盘类型（模板参数）
#include "contracts.hpp"           // Twist / WheelSpeeds / Pose
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"
#include "wheel.hpp"

struct ChassisLoopConfig {
    float control_dt_ = 0.001f;      // s，控制环周期（调用方也给 dt，两处必须一致）

    // ── 底盘几何（决定逆/正解）──
    float wheel_radius_   = 0.03f;   // m
    float half_wheelbase_ = 0.10f;   // m，lx
    float half_track_     = 0.12f;   // m，ly

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
    // IO 从外面注入 —— 与 Wheel 的 MeasureSpeedFn / SetPwmFn 同型
    using MeasureSpeedFn = float (*)(uint8_t wheel_id, float dt);
    using SetPwmFn       = void  (*)(uint8_t wheel_id, int16_t pwm);

    ChassisLoop(const ChassisLoopConfig& cfg,
                MeasureSpeedFn measure_speed,
                SetPwmFn       set_pwm);

    // ── 上游接口 ──
    void set_cmd(const Twist& cmd);              // 本拍想跑的车体速度

    // ── 唯一的心跳 ──
    //   dt  = 实测周期（不许用标称值）
    //   now = 绝对时间基（ms），由调用方提供 —— 它才是 IO 的拥有者
    void tick(float dt, uint32_t now);

    // ── 只读口（见 §6，P19 的数据入口，不是可选的）──
    Pose  pose() const;
    float yaw_odo() const;                       // 连续角 = yaw_ref()
    Twist twist() const;                         // 本拍 FK 输出（标定后）
    Twist cmd() const;                           // 本拍被要求的
    float wheel_speed(uint8_t i) const;          // 单轮实测
    float wheel_target(uint8_t i) const;         // 单轮目标
};
```

**注意 `tick(dt, now)` 而不是 `tick(dt)`** —— 装配层**不许自己发明时间基**。
时间基是 IO 的产物，它的拥有者是调用方（固件的定时器 / PC 的 `hal_host::advance`）。

**轮数（已定案）**：内部构造 4 个 `Wheel`（`w0_..w3_` + `Wheel* wheels_[4]`），
`tick()` 里按 `target_ws_.count_` 决定**实际用前几个**：

```cpp
const uint8_t n = target_ws_.count_;     // 底盘说几轮就几轮（diff=2 / omni=3 / mec=4）
for (uint8_t i = 0; i < n; ++i) { wheels_[i]->set_cmd(...); wheels_[i]->update(dt); }
```

没被用到的 `Wheel` 永远不 `update()` → 不会写 PWM、不会调 `MeasureSpeedFn` ✓。

## 5. `tick()` 的执行顺序（顺序本身是设计的一部分）

```
① 限幅      limiter_.limit(cmd_, dt)         ← 先把上游野值收进物理可行范围
② 逆解      chassis_.inverse_kinematics()    ← 车体 Twist → N 轮目标
③ 下发+执行 wheels_[i]->set_cmd() / update() ← S 曲线 → PID → SetPwmFn
④ 测量      wheels_[i]->get_speed()          ← MeasureSpeedFn 读编码器
⑤ 正解      chassis_.forward_kinematics()    ← N 轮 → 车体 Twist
⑥ 里程计    odom_.update(body, dt, now, twist_, &meas)
⑦ （离开本层）姿态融合                        ← 使用者：observe_heading(loop.yaw_odo())
```

**为什么这个顺序**：

| 步 | 不许调换的理由 |
|---|---|
| ① 在最前 | 后面所有环节都要靠"已限幅"这个前提才安全 |
| ③ 在 ② 之后 | 逆解的输出是轮控的输入 |
| ④ 在 ③ 之后 | `update()` 内部**已经**通过 `MeasureSpeedFn` 读了编码器，`get_speed()` 取的是同一拍的值 |
| ⑤ 在 ④ 之后 | 正解吃的是实测轮速 |
| ⑥ 在 ⑤ 之后 | 里程计吃的是正解输出 |
| ⑦ 离开本层 | 它吃 `yaw_odo()`，所以必须在 ⑥ 之后 —— 由使用者保证（**这是唯一的顺序责任外移**） |

## 6. 只读口 —— 这不是"顺便加的"

**P19（一致性残差监测）的全部数据原料都从这里出**：

| 只读口 | 谁要用 |
|---|---|
| `twist()` | `wz` 残差的**左半边**：`FK(轮速).wz − gyro_z` |
| `cmd()` | 堵转 / 执行器故障检测（`cmd` vs `twist`） |
| `wheel_speed(i)` / `wheel_target(i)` | **轮间一致性**（单轮打滑 / 缺气） |
| `pose()` / `yaw_odo()` | 融合钩子；长窗残差 |
| `tick`（随 `odometry` 的 `SampleSink`） | 时间戳对齐 |

**所以只读口是"接口的一部分"，不是调试附属品** —— 少了它，P19 无从下手。

## 7. 验收（可机器判）

**把 `~/Develop/Workspace/KND_Trial/firmware/` 里的 `app.cpp` 换成用本组件，
仿真输出必须逐位不变：**

```
改造前：最终: pose=(0.3425, 0.1085) yaw_odo=0.2574 (0.04 圈) | imu 拒绝样本=0
        四轮最终转速: 0.52 1.40 1.04 0.88
```

**行为保持**是重构的唯一判据。

另外**三种用法**照旧要过：

1. 聚合构建（`ctest` 总数 +1）
2. `cmake -S chassis_loop` 单库构建（开发模式出测试）
3. 被固件消费时**进入库模式、零泄漏**

## 8. 已定案（2026-09-15 · 用户：按推荐）

| # | 决策 | 结论 | 一句话理由 |
|---|---|---|---|
| **1** | **名字** | ✅ **`chassis_loop` / `ChassisLoop`** | "loop" = 闭环 + 时间基，正是它的本质；且不与 `control/` 目录歧义。`ChassisController`（`THEORY.md` 历史名）被否 —— "Controller" **名不副实**，它还含里程计 |
| **2** | **位置** | ✅ **仓库根级**（与 `contracts/` `kinematics/` `odometry/` 平级） | 它编排的不只是 `control/`，放在 `control/` 下会误导 |
| **3** | **轮数** | ✅ **构造 4 个 `Wheel`，按 `WheelSpeeds.count_` 实际使用** | 差速（2 轮）只用到前 2 个，不用改代码。`Wheel` 无默认构造，N 泛化要 `index_sequence` 技巧 —— 与「古法编程」节奏不合，留到真需要时 |
| **4** | **预留 `set_correction(δcmd)`** | ✅ **不做** | `ARCHITECTURE.md` §3.3 自己写着「这一层在两个设计里都是空白」。**为没设计出来的东西预留接口 = 猜**（同 `Estimator` 的 P0-1） |

## 9. 落地顺序（建议）

```
① ~~你拍 §8 的 4 条~~ ✅ **已定案（2026-09-15）**
② 你手写 ⬜ `chassis_loop/inc/chassis_loop.hpp` + ⬜ `chassis_loop/src/chassis_loop.cpp` + `CMakeLists.txt`
   （照 §4 的契约；顺序照 §5）
③ AI 写 ⬜ `chassis_loop/test/test_chassis_loop.cpp`
   —— 测行为不变式：tick 顺序、dt 穿透、只读口自洽、限幅生效、reset 语义
④ 三档编译 + 全量 ctest + 文档门禁 + ci_local
⑤ 改 KND_Trial firmware 的 app.cpp 用本组件 → 仿真输出逐位比对（§7）
⑥ 本文件进 trash/
```

## 10. 一个必须记住的坑（来自 foucault 的 P0-1）

**别在构造函数里写死配置。**

```cpp
// ✗ 反面教材（foucault 的 Estimator 就是这么写的，于是 Estimator<EKF> 编不过）
explicit ChassisLoop(...) : limiter_(1.5f, 1.5f, 4.0f) {}

// ✓ 配置从外面进来
ChassisLoop(const ChassisLoopConfig& cfg, ...)
    : limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_) {}
```

**判据**（`../AGENTS.md` §5.3）：**接口里出现「替使用者决定」就是越界。**
