---
class: status
generated: false
---
> **类：C 状态** —— **跟代码变**。代码地图：只记「东西在哪、叫什么」。
> **契约与决策**（B 事实）在 [`DESIGN.md`](DESIGN.md) —— 这里不重复，冲突以那边为准。
> 文档体系与写作规则：[`../../docs/README.md`](../../docs/README.md)

# IMPL.md — chassis_loop 代码地图

## 1. 文件布局

| 路径 | 是什么 |
|---|---|
| `inc/chassis_loop.hpp` | **编排层**：`ChassisLoopConfig` + `ChassisLoop<ActuatorSet>`（命名空间 `lunokhod::chassis_loop`） |
| `inc/wheel_set.hpp` | **执行器组**（接缝的一个实现）：`WheelSetConfig` + `WheelSet<Chassis>` + `WheelLoop` 别名（同命名空间） |
| `test/test_chassis_loop.cpp` | 9 组 / 64 条断言（AI 写，出题人） |
| `CMakeLists.txt` | `chassis_loop` = **INTERFACE** 库（**没有 `.cpp`** —— 全模板，见 DESIGN.md §9 F2） |
| `docs/DESIGN.md` | B 事实：契约 / 决策 D1~D9 / 边界 |
| `docs/log/ACCEPTANCE.md` | A 日志：首批落地与验收记录 |
| `docs/IMPL.md` | 本文件（C 状态） |

## 2. `ChassisLoopConfig` 的字段与消费者

**两份配置，各归各家**（DESIGN.md §5.2）：

| 配置 | 字段 | 谁消费 |
|---|---|---|
| `ChassisLoopConfig`（编排层） | `acc_vx_` / `acc_vy_` / `acc_wz_` | `TwistAccLimiter` |
| 〃 | `twist_scale_` | `Odometry` |
| `WheelSetConfig`（执行器组） | `pid_` | 6 个 `Wheel` |
| 〃 | `planner_` | 6 个 `Wheel` |

**编排层**里**没有**底盘几何（归 `Chassis`，§8.2 **D6**）· **没有** `PID`/`planner`（归执行器组）· **没有** `control_dt_`（**D4**）。

## 3. 两个类的成员（实际名字）

### 3.1 `ChassisLoop<ActuatorSet>`

| 成员 | 类型 | 说明 |
|---|---|---|
| `actuators_` | `ActuatorSet` | 构造时从外面注入的执行器组（本轮 = `WheelSet<Chassis>`） |
| `limiter_` | `TwistAccLimiter` | |
| `odom_` | `Odometry` | |
| `t_cmd_in_` | `Twist` | ① 上游要求（**限幅前**） |
| `t_cmd_final_` | `Twist` | ② 限幅后、**真正下发**的那一份 |
| `ws_target_` | `WheelSpeeds` | ② 执行器目标（执行器组已把尾巴堵 0，DESIGN.md §9 F1） |
| `ws_meas_` | `WheelSpeeds` | ④ 实测（零初始化；`wheel_speed()` 读它） |
| `twist_meas_` | `Twist` | ⑤ 正解输出（**未**乘 `twist_scale_`） |

### 3.2 `WheelSet<Chassis>`

| 成员 | 类型 | 说明 |
|---|---|---|
| `chassis_` | `Chassis` | 底盘（几何归它，§8.2 **D6**） |
| `wheels_` | `std::array<wheel::Wheel, 6>` | **容量 = 契约容量**（**D10**）；活跃数见设定值的 `count_` |

**契约 = 五个动作**：`inverse` / `apply` / `measure` / `forward` / `effort(i)`（第 5 个见 **D13**）。
`wheel` 组件的类型自 **P29** 起都在 `namespace wheel` 里（`wheel::Wheel` / `wheel::PIDConfig` / …）。

> **除轮子外没有状态** —— 活跃几个轮子这件事由 `apply` / `measure` 的**入参**说了算
> （2026-09-17 定案，DESIGN.md §5.2）。**不要**改回 `Wheel* wheels_[6]`：自引用指针
> 一旦按值注入就会悬空（组件账本第 5 条）。

## 4. 公开口 → 契约位置

| 公开口 | 语义 | 契约 |
|---|---|---|
| `set_cmd(Twist)` | 上游要求 | DESIGN.md §5 |
| `set_sink(SampleSink, void*)` | **转发**给 `odom_`（P19 的时间戳原料） | §5 · §8.2 **D8** |
| `tick(float dt, uint32_t now)` | 唯一心跳；`dt`/`now` **原样**分发，不自己造 | §6 |
| `pose()` / `yaw_odo()` | 位姿（wrap）／连续角（融合钩子吃它） | §7 |
| `twist()` | ④ 正解输出 | §5.1 · §8.2 **D1/D3** |
| `cmd()` | ② 限幅后、真正下发的 | §5.1 · §8.2 **D2** |
| `wheel_speed(i)` | 单轮**实测** | §7 |
| `wheel_target(i)` | 单轮**目标**（仅 `i < count_` 有意义，§9 F3） | §7 |
| `wheel_effort(i)` | 单轮**最近一次算出的 effort**（`int16_t`；未用到的轮 = 0） | §7 · §8.2 **D13** |

## 5. `tick()` 六步 → 代码符号

| 步 | 代码 |
|---|---|
| ① 限幅 | `t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_` |
| ② 目标 | `ws_target_ = actuators_.inverse(t_cmd_final_)` |
| ③ 下发+执行 | `actuators_.apply(ws_target_, dt)`（执行器组内部只动前 `count_` 个） |
| ④ 测量 | `ws_meas_ = actuators_.measure(ws_target_)` |
| ⑤ 正解 | `twist_meas_ = actuators_.forward(ws_meas_)` |
| ⑥ 里程计 | `odom_.update(twist_meas_, dt, now, t_cmd_final_, &ws_meas_)` |

顺序为何不能换：DESIGN.md §6。

## 6. 测试

`test_chassis_loop.cpp` 9 组：逆解手算锚点 · 限幅性质与到位拍数 · `cmd()`=限幅后且限幅先于逆解 ·
**边界 N（三轮 / 六轮）** ·
执行顺序与未用轮不被触碰 · `dt`/`now` 穿透 · 互逆往返 · 目标真的下发到轮子 ·
**接缝一致性**（假执行器组 `StubSet`，证明 `ActuatorSet` 这个模板参数真被用而非装饰）·
单轮 `effort` 暴露（与回调实际收到的值比对，含"未用到的轮 = 0"）。
oracle 与容差推导见施工单 §3；判别力审计（13 个变异全部变红）见施工单 §4。

## 7. 挂载点

- 根 `CMakeLists.txt` **末尾**：`add_subdirectory(chassis_loop)` —— 它依赖其余全部，必须最后。
- 聚合 `ctest`：**9/9**（本组件贡献 1 个 `test_chassis_loop`）。
- 被消费：`LUNOKHOD_DEV_BUILD=OFF` 时只出库，零泄漏（实测 0 个 test/example 可执行文件）。
