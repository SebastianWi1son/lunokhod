---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。
> chassis_loop（**装配层**）设计权威：范围判据 / 职责 / 接口契约 / 决策 D1~D9 / 已知边界。
> 文档体系与写作规则：[`../../docs/README.md`](../../docs/README.md)

# DESIGN.md — lunokhod/chassis_loop 组件设计

> 状态：**接口已冻结（2026-09-16）**，D1~D9 全部拍板；**代码待手写**（⬜ `inc/chassis_loop.hpp`）。
> 施工参考（**已归档，不是代码源**）：`../../trash/WORK_CHASSIS_LOOP.md`
> 验收记录：[`log/ACCEPTANCE.md`](log/ACCEPTANCE.md)
> 历史提案：[`../../kinematics/docs/THEORY.md`](../../kinematics/docs/THEORY.md) §5.2 的 `ChassisController`（已否决，见 §8.1）
> 挂起事务：[`../../docs/TODO.md`](../../docs/TODO.md) **P20**（本组件）· **P22**（遗留的 `reset()` 语义）

## 0. 一句话

**把 lunokhod 的四块积木接到【同一个时间基】上，按固定顺序跑一遍。**
它**不生产数据，只搬运和排列** —— 所有算法都在被它调用的库里。

## 1. 为什么存在（证据）

同一条装配链，此前**手写了两遍**（`~/Develop/Workspace/fw_poc/` 的 `main.cpp` 110 行、
`~/Develop/Workspace/KND_Trial/firmware/` 的 `app.cpp` 68 行）。链里有**易错知识**：

1. `tick` 的**顺序**（限幅必须在逆解前、里程计必须在正解后）
2. `dt` 必须用**实测值**，不许用标称值（喂标称 1 ms 而实际 1.2 ms = **系统性多算 20%**）
3. 姿态融合钩子必须在**里程计之后**（它要吃本拍最新的航向）
4. 参数必须聚合（`PIDConfig` / `SmoothPlannerConfig` 不许散落在函数体里）

**每重写一遍就有一次写错的机会。** 它同时是整条感知线（`TODO.md` P19）的**取样点**：
§7 的只读口就是那儿的数据入口。

## 2. 范围（钉死）

```
set_cmd(Twist) → 限幅 → 逆解 → N×Wheel → 正解 → 里程计 → pose / yaw_odo
```

**只做下行链 + 里程计。**

### 判据（越界自查）

> **装配层只吃「已算好的量」（`Twist` / `dt` / `now`），不碰 IO。**
> 一旦它需要知道「命令从哪来」「IMU 在哪读」 → **那不是装配层，是应用层。**

### 不做的两件事（各有理由）

| 不做 | 为什么 |
|---|---|
| **姿态融合** | 碰了就要 `#include "estimator.hpp"` → **lunokhod 依赖 foucault** ✗（见 `../../AGENTS.md` §5.3「跨库只传标量」）。融合由使用者接一根线：`ahrs.observe_heading(loop.yaw_odo())` |
| **修正输入**（`δcmd`） | `../../docs/ARCHITECTURE.md` §3.3 自己写着「这一层在两个设计里都是空白」。**为没设计出来的东西预留接口 = 猜**（同 `foucault::Estimator<EKF>` 的 P0-1） |

## 3. 职责（一件事 + 五个不做）

**「编排」= 六条接线 + 它们的先后**：

```cpp
cmd_out_    = limiter_.limit(cmd_in_, dt).out_;        // ① 中间量
target_ws_  = chassis_.inverse_kinematics(cmd_out_);   // ② 谁吃 cmd_out_
wheels_[i]->set_cmd(target_ws_.values_[i]);            // ③ 谁吃 target_ws_
meas.values_[i] = wheels_[i]->get_speed();             // ④ 谁产出 meas
twist_meas_ = chassis_.forward_kinematics(meas);       // ⑤ 谁吃 meas
odom_.update(twist_meas_, dt, now, cmd_out_, &meas);   // ⑥ 谁吃 twist_meas_
```

四个库各自只看自己那一格 —— `Wheel` 不知道目标来自逆解，`Odometry` 不知道 `twist` 来自实测轮速。
**装配层固定的是「每格之间怎么接」。**

| 职责 | 内容 | 是「替使用者决定」吗 |
|---|---|---|
| **持有** | 拥有 `chassis_` / `limiter_` / `odom_` / 4×`Wheel` 实例 | 否 |
| **编排** | 上面六条接线 + 先后 | 否（顺序由语义规定，不是选择） |
| **分发时间基** | **同一份** `dt` / `now` 给限幅、每个 `Wheel`、里程计 | 否 |
| **轮数派发** | 读 `target_ws_.count_`，只跑前 n 个 `Wheel` | 否（**问**底盘要事实） |
| **观测口** | 只读口（§7）+ `set_sink` 转发 | 否 |

**五个不做**：不产生时间基 · 不碰 IO · 不含算法 · 不做判定 · 不决定命令从哪来。

### 时间基：它**不是**一块代码，是**一条数据**

`ChassisLoop` 里**没有一行**读时钟、算 `dt`、插值。时间基的**拥有者是调用方**
（固件定时器 / `hal::tick()`），装配层只是**唯一的分发者**（`tick(dt, now)` 的两个入参）。
所以「时间基」不可被拆成一个角色 —— 它是参数。

### 何时该拆成两个角色（触发条件）

成熟框架里「拥有周期」与「某条链的实现」常分属两个类
（ros2_control：`ControllerManager` 按序 `update()` 各 controller；`diff_drive_controller` 只管一条底盘链）。
**v1 不拆**（见 §8.2 D9），触发条件是：**出现第二件需要同期执行的事** ——
云台（`TODO.md` P8）、FOC 轴（P7）要与底盘共用一个周期。**那时**再拆，
而且调度者的接口要**由第二个消费者的需求来定**（块列表？顺序/优先级？时间源？）。

## 4. 位置与依赖

```
lunokhod/
├── contracts/         ← 最底层
├── chassis_loop/      ← 本组件（在 control/ 之外 —— 它编排的不只是控制）
├── kinematics/
├── odometry/
└── control/
    ├── twist_acc_limiter/
    └── wheel/
```

依赖方向（**单向、无环**，本组件在**最上层**）：

```
contracts ← kinematics ←┐
contracts ← odometry   ←┼← chassis_loop ← （固件 / 应用）
contracts ← twist_acc_limiter ←┘
contracts ← wheel      ←┘
```

## 5. 对外接口（契约）

```cpp
// inc/chassis_loop.hpp

#include "chassis.hpp"            // kinematics：底盘类型（模板参数）
#include "contracts.hpp"           // Twist / WheelSpeeds / Pose
#include "odometry.hpp"
#include "twist_acc_limiter.hpp"
#include "wheel.hpp"

// 参数聚合（常量与参数不许散落在函数体里）
// 注意：这里【没有】底盘几何 —— 几何属于 Chassis（§8.2 D6）
//       也【没有】control_dt_ —— 时间基唯一来源是 tick(dt, now)（§8.2 D4）
struct ChassisLoopConfig {
    float acc_vx_ = 1.5f;            // m/s²，Twist 空间加速度限幅
    float acc_vy_ = 1.5f;
    float acc_wz_ = 4.0f;            // rad/s²

    PIDConfig           pid_{};      // 轮控
    SmoothPlannerConfig planner_{};

    float twist_scale_ = 1.0f;       // 里程计标定系数（硬件属性）
};

template <typename Chassis>
class ChassisLoop {
public:
    using MeasureSpeedFn = float (*)(uint8_t wheel_id, float dt);
    using SetPwmFn       = void  (*)(uint8_t wheel_id, int16_t pwm);

    // 底盘【从外面注入】—— 几何是底盘自己的事（§8.2 D6）
    ChassisLoop(const ChassisLoopConfig& cfg,
                const Chassis&          chassis,
                MeasureSpeedFn          measure_speed,
                SetPwmFn                set_pwm);

    void set_cmd(const Twist& cmd);              // 上游：本拍想跑的车体速度
    void set_sink(SampleSink sink, void* ctx = nullptr);   // §8.2 D8

    // 唯一的心跳：dt = 实测周期（不许标称值）；now = 绝对时间基（ms，调用方给）
    void tick(float dt, uint32_t now);

    // 只读口（§7）
    Pose  pose() const;
    float yaw_odo() const;                       // 连续角
    Twist twist() const;                         // ⑤ 正解输出（**未**标定）
    Twist cmd() const;                           // ①【限幅后】、真正下发的
    float wheel_speed(uint8_t i) const;
    float wheel_target(uint8_t i) const;         // 仅 i < count_ 有意义
};
```

**库类型 = INTERFACE**（全模板 + POD 配置，**没有 `.cpp`**，见 §9 F2）。

**轮数（决策 §8.1）**：内部构造 4 个 `Wheel`（`w0_..w3_` + `Wheel* wheels_[4]`），
`tick()` 里按 `target_ws_.count_` 决定**实际用前几个**（diff=2 / omni=3 / mec=4）。
没被用到的 `Wheel` 永远不 `update()` → 不会写 PWM、不会调 `MeasureSpeedFn` ✓。

### 5.1 三个 `Twist` 的名字（2026-09-16 定案）

链上同时活着**三个** `Twist`，必须钉死（旧接口块曾让 `twist()` 同时指①和③）：

| 量 | 名字 | 谁拥有 | 备注 |
|---|---|---|---|
| ① 上游要求（**限幅前**） | （内部 `cmd_in_`） | **调用方**（它调的 `set_cmd`） | **不开只读口** —— 调用方自己就有 |
| ② 真正下发（**限幅后**） | `cmd()` | 装配层 | 喂 odometry 的 `cmd` 入参就是它 |
| ③ 正解反推 `FK(实测轮速)` | `twist()` | 装配层 | **不乘** `twist_scale_`（标定是 odometry 内部的事） |

**为什么 ① 不能当 L0 残差的左半边**：`TwistAccLimiter::ramp()` 未饱和时**原值返回**，
所以 ①−② 只在限幅饱和时非零 —— 拿它做 `cmd vs twist` 会退化成「限幅饱和指示器」，
不是 `TODO.md` P19 §7 要的「执行器故障 / 轮间一致性」。

**命名原则**：**按链条位置命名，不按词命名**。每级都可插拔，所以「`cmd`」这个词的含义
会随装配漂移；位置不会。

## 6. `tick()` 的执行顺序（顺序本身是设计的一部分）

```
① 限幅      limiter_.limit(cmd_in_, dt)      ← 先把上游野值收进物理可行范围
② 逆解      chassis_.inverse_kinematics()    ← 车体 Twist → N 轮目标
③ 下发+执行 wheels_[i]->set_cmd() / update() ← S 曲线 → PID → SetPwmFn
④ 测量      wheels_[i]->get_speed()          ← MeasureSpeedFn 读编码器
⑤ 正解      chassis_.forward_kinematics()    ← N 轮 → 车体 Twist
⑥ 里程计    odom_.update(twist(), dt, now, cmd(), &meas)
⑦ （离开本层）姿态融合                        ← 使用者：observe_heading(loop.yaw_odo())
```

| 步 | 不许调换的理由 |
|---|---|
| ① 在最前 | 后面所有环节都要靠「已限幅」这个前提才安全 |
| ③ 在 ② 之后 | 逆解的输出是轮控的输入 |
| ④ 在 ③ 之后 | `update()` 内部**已经**通过 `MeasureSpeedFn` 读了编码器，`get_speed()` 取的是同一拍的值 |
| ⑤ 在 ④ 之后 | 正解吃的是实测轮速 |
| ⑥ 在 ⑤ 之后 | 里程计吃的是正解输出 |
| ⑦ 离开本层 | 它吃 `yaw_odo()`，所以必须在 ⑥ 之后 —— 由使用者保证（**这是唯一的顺序责任外移**） |

## 7. 只读口 —— 这不是"顺便加的"

**`TODO.md` P19（一致性残差监测）的全部数据原料都从这里出**：

| 只读口 | 谁要用 |
|---|---|
| `twist()` | `wz` 残差的**左半边**：`FK(轮速).wz − gyro_z` |
| `cmd()` | 堵转 / 执行器故障检测（`cmd` vs `twist`） |
| `wheel_speed(i)` / `wheel_target(i)` | **轮间一致性**（单轮打滑 / 缺气） |
| `pose()` / `yaw_odo()` | 融合钩子；长窗残差 |
| `set_sink`（转发 `SampleSink`） | **时间戳对齐**（逐拍记录 + `tick_`） |

**所以只读口是"接口的一部分"，不是调试附属品** —— 少了它，P19 无从下手。

## 8. 决策（D1~D9 + 名字/位置/轮数）

### 8.1 早期三条（2026-09-15）

| # | 决策 | 结论 | 一句话理由 |
|---|---|---|---|
| **名字** | **`chassis_loop` / `ChassisLoop`** | ✅ | ⚠ **2026-09-16 更正否掉 `ChassisController` 的理由**：原写「名不副实 —— 它还含里程计」**站不住**（ros2_control 的 `diff_drive_controller` / `mecanum_drive_controller` 也含里程计）。真理由是**库内 `controller` 一词已被控制律占用**（`PID` / `SmoothPlanner` / `TwistAccLimiter` / `Wheel`），叫 `ChassisController` 会让人以为 PID 在这个库里 —— 而它一行控制律都没有。"loop" = 闭环 + 时间基，也是嵌入式最普遍的叫法（`loop()` / `Run()` / `update()` / AUTOSAR Runnable / PX4 WorkItem）
| **位置** | **仓库根级** | ✅ | 它编排的不只是 `control/`，放在 `control/` 下会误导 |
| **轮数** | **构造 4 个 `Wheel`，按 `WheelSpeeds.count_` 实际使用** | ✅ | 差速（2 轮）只用到前 2 个，不用改代码。N 泛化要 `index_sequence` 技巧 —— 与「古法编程」节奏不合，留到真需要时 |
| **`set_correction(δcmd)`** | **不做** | ✅ | 「为没设计出来的东西预留接口 = 猜」 |

### 8.2 接口层 D1~D9（2026-09-16）

| # | 问题 | 结论 |
|---|---|---|
| **D1** | `twist()` 指 ①上游要求还是 ③FK 输出 | ③（见 §5.1） |
| **D2** | `cmd()` 是限幅前还是限幅后 | **限幅后**（真正下发的那一份） |
| **D3** | `twist()` 要不要乘 `twist_scale_` | **不乘**（标定是 odometry 内部的事） |
| **D4** | `control_dt_` 要不要留在配置里 | **删** —— 它是第二个时间源，正是 §1 第 2 条警告的那个 bug 的钩子 |
| **D5** | 测量缓冲 `WheelSpeeds` 怎么初始化 | `WheelSpeeds meas{};` + 显式写回 `count_`（未用槽留 0） |
| **D6** | **底盘几何从哪来** | **(a) 底盘实例从外面注入** —— `ChassisLoop(cfg, MecanumDrive(0.10f, 0.12f, 0.03f), ...)` |
| **D7** | 要不要 `reset()` | **v1 不做** —— 理由见下 |
| **D8** | 要不要转发 `set_sink` | **要** —— 一行，把 odometry 已有的逐拍记录（含 `tick_`）开给调用方 |
| **D9** | 要不要拆成两个角色（调度者 + 底盘链） | **v1 不拆** —— 见 §3「何时该拆」 |

**D6 为什么是 (a)**：三种底盘构造签名不同 —— `MecanumDrive(lx, ly, r)` ·
`DiffDrive(wb, r)`（`wb` 是**全**轮距）· `OmniDrive(wn, cr, gamma, wr)`。
让库内构造，`ChassisLoop<DiffDrive>` **编译不过**（实测：`no matching function for call to
DiffDrive::DiffDrive(const float&, const float&, const float&)`）。
原接口块之所以看着没问题，是因为 KND_Trial 的 `App` **不是模板**（写死麦轮）。
(a) 并不是"不能配置后构造"，而是**那一行写在调用方**（`app.cpp`）。
附带好处：几何归底盘（`AGENTS.md` §5.3「我需要什么就声明什么」），
模板参数**有实际作用**，不会退化成装饰。

**D7 为什么不做**：`reset()` **不是**热切换配置（换参数需要 `set_config()`，那个不做）。
它只清**内部状态**（`TwistAccLimiter::prev_` / `PID::integral_` / `SmoothPlanner::prev_` /
`Wheel::speed_cmd_` / `Odometry` 位姿），而这些积木**都有**清零入口（`reset()` / `stop()`）——
但都是 private 成员，不开口调用方**一个都够不着**。**v1 不做的理由**：急停/复位路径还没设计，
现在定语义就是猜（例如「要不要顺带 `stop()` 那 4 个 `Wheel`」两难：写 PWM 0 会碰**未用到的轮子**，
违反轮数决策；只停前 n 个又不行 —— `reset()` 可能在第一次 `tick()` 之前被调，那时 `count_` 还是 0）。
**⇒ 已记 `TODO.md` P22。**

**D8 为什么转发**：`odom_` 是 private 成员，调用方够不着 → 装不上 `SampleSink` → 拿不到逐拍记录。
只读口是**时刻快照**（不带时间戳、采样率不同步会丢拍），而 P19 要的正是「逐拍记录 + `tick_` 时间戳」
拿去和 foucault 的 IMU 日志对齐。成本 = 一行。

**D9 为什么 v1 不拆**：见 §3（拆出来的"调度者"是空的：没有状态、没有算法，只有一个调用序列）。

## 9. 已知边界与发现

**F1 —— `kinematics::jacobian_apply()` 只填前 `count_` 个槽，尾巴是 indeterminate**

```cpp
WheelSpeeds out_ws;          // ← 没有零初始化
```

`MecanumDrive::forward_impl()` 却**无条件**读 `values_[0..3]`。当前三种底盘的 `count_`
与读取范围一致，所以还不构成 UB；但「坏数据静默外流」违反 `../../AGENTS.md` 铁律 5：
P19 采 CSV 时，没用到的那几槽就是脏数据。
**处置**：装配层在逆解后**堵一次**（`for (i = n; i < 6; ++i) target_ws_.values_[i] = 0.0f;`），
**不动 kinematics 源码**（铁律 §2.1）。

**F2 —— 库类型是 INTERFACE，没有 ⬜ `src/chassis_loop.cpp`**

本组件全是模板 + POD 配置，所有代码必须在头文件里。`AGENTS.md` §5.1 的
「有 `.cpp` 就必须进 STATIC 库」这条**不适用**（一个 `.cpp` 都没有）。

**F3 —— `wheel_target(i)` 只在 `i < count_` 内有意义**

`WheelSpeeds.values_` 只有前 `count_` 个有效（见 F1）。调用方若要遍历，请以上一次
`tick()` 之后的 `count_` 为界（装配层内部只读 `[0, count_)`）。

## 10. 设计准则（一个必须记住的坑）

**别在构造函数里写死配置。**

```cpp
// ✗ 反面教材（foucault 的 Estimator 就是这么写的，于是 Estimator<EKF> 编不过）
explicit ChassisLoop(...) : limiter_(1.5f, 1.5f, 4.0f) {}

// ✓ 配置从外面进来
ChassisLoop(const ChassisLoopConfig& cfg, ...)
    : limiter_(cfg.acc_vx_, cfg.acc_vy_, cfg.acc_wz_) {}
```

**判据**（`../../AGENTS.md` §5.3）：**接口里出现「替使用者决定」就是越界。**

## 11. 验收（可机器判）

**把 `~/Develop/Workspace/KND_Trial/firmware/` 里的 `app.cpp` 换成用本组件，
仿真输出必须逐位不变**：

```
改造前：最终: pose=(0.3425, 0.1085) yaw_odo=0.2574 (0.04 圈) | imu 拒绝样本=0
        四轮最终转速: 0.52 1.40 1.04 0.88
```

**行为保持**是重构的唯一判据。

另外**三种用法**照旧要过：

1. 聚合构建（`ctest` 总数 +1）
2. `cmake -S chassis_loop` 单库构建（开发模式出测试）
3. 被固件消费时**进入库模式、零泄漏**

**测试的 oracle**（详见 [`log/ACCEPTANCE.md`](log/ACCEPTANCE.md) 与 `trash/WORK_CHASSIS_LOOP.md`）：
装配层是**接线层**，**不能拿它自己的输出算期望值**。可用的是
**金标**（另一份代码算出来的仿真输出）· **手算锚点**（几何 + 阶跃命令 → 有理数）·
**性质**（互逆 / `|Δtwist| ≤ acc·dt` / 只读口自洽）+ 两条装配层独有的不变式
（**调用顺序**、**`tick` 不发明时间基**）。
