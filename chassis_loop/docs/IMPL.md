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
| `inc/chassis_loop.hpp` | **全部代码**（84 行）：`ChassisLoopConfig` + `ChassisLoop<Chassis>` |
| `test/test_chassis_loop.cpp` | 7 组 / 50 条断言（AI 写，出题人） |
| `CMakeLists.txt` | `chassis_loop` = **INTERFACE** 库（**没有 `.cpp`** —— 全模板，见 DESIGN.md §9 F2） |
| `docs/DESIGN.md` | B 事实：契约 / 决策 D1~D9 / 边界 |
| `docs/log/ACCEPTANCE.md` | A 日志：首批落地与验收记录 |
| `docs/IMPL.md` | 本文件（C 状态） |

## 2. `ChassisLoopConfig` 的字段与消费者

| 字段 | 谁消费 |
|---|---|
| `acc_vx_` / `acc_vy_` / `acc_wz_` | `TwistAccLimiter` |
| `pid_` / `planner_` | 4 个 `Wheel` |
| `twist_scale_` | `Odometry` |

**没有**底盘几何（归 `Chassis`，DESIGN.md §8.2 **D6**）；**没有** `control_dt_`（**D4**）。

## 3. `ChassisLoop<Chassis>` 的成员（实际名字）

| 成员 | 类型 | 说明 |
|---|---|---|
| `chassis_` | `Chassis` | 构造时从外面注入的那一份 |
| `limiter_` | `TwistAccLimiter` | |
| `odom_` | `Odometry` | |
| `w0_` `w1_` `w2_` `w3_` | `Wheel` | **永远构造 4 个**（决策：轮数） |
| `wheels_` | `Wheel*[4]` | 指向上面四个；`tick()` 里只用前 `count_` 个 |
| `t_cmd_in_` | `Twist` | ① 上游要求（**限幅前**） |
| `t_cmd_final_` | `Twist` | ② 限幅后、**真正下发**的那一份 |
| `ws_target_` | `WheelSpeeds` | ③ 逆解输出的**单轮目标**（尾巴恒为 0，见 DESIGN.md §9 F1） |
| `twist_meas_` | `Twist` | ④ 正解输出（**未**乘 `twist_scale_`） |

> 内部名与 DESIGN.md 参考实现里的 `cmd_in_` / `cmd_out_` / `target_ws_` **只是叫法不同**，语义一一对应
> —— 内部名不属于契约（命名家族：车体指令 = `cmd`，单轮目标 = `target`，单轮实测 = `speed`）。

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

## 5. `tick()` 六步 → 代码符号

| 步 | 代码 |
|---|---|
| ① 限幅 | `t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_` |
| ② 逆解 | `ws_target_ = chassis_.inverse_kinematics(t_cmd_final_)`，然后把 `values_[wn..5]` 堵成 0 |
| ③ 下发+执行 | `wheels_[i]->set_cmd(ws_target_.values_[i])` / `wheels_[i]->update(dt)`（只用前 `wn` 个） |
| ④ 测量 | `WheelSpeeds meas{}`（零初始化）→ `meas.values_[i] = wheels_[i]->get_speed()` |
| ⑤ 正解 | `twist_meas_ = chassis_.forward_kinematics(meas)` |
| ⑥ 里程计 | `odom_.update(twist_meas_, dt, now, t_cmd_final_, &meas)` |

顺序为何不能换：DESIGN.md §6。

## 6. 测试

`test_chassis_loop.cpp` 7 组：逆解手算锚点 · 限幅性质与到位拍数 · `cmd()`=限幅后且限幅先于逆解 ·
执行顺序与未用轮不被触碰 · `dt`/`now` 穿透 · 互逆往返 · 目标真的下发到轮子。
oracle 与容差推导见施工单 §3；判别力审计（13 个变异全部变红）见施工单 §4。

## 7. 挂载点

- 根 `CMakeLists.txt` **末尾**：`add_subdirectory(chassis_loop)` —— 它依赖其余全部，必须最后。
- 聚合 `ctest`：**9/9**（本组件贡献 1 个 `test_chassis_loop`）。
- 被消费：`LUNOKHOD_DEV_BUILD=OFF` 时只出库，零泄漏（实测 0 个 test/example 可执行文件）。
