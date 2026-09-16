---
class: log
generated: false
---
> **类：A 日志（append-only）** —— 带日期，**只增不改**，不代表当前状态。
> 当前状态见 [`../../docs/TODO.md`](../../docs/TODO.md)（P20/P22/P24）与 [`../IMPL.md`](../IMPL.md)。
> 这份是 **chassis_loop 首批落地的验收日志**：结论 + 数字 + 踩的坑。

# ACCEPTANCE.md — chassis_loop 首批落地与验收（2026-09-16）

## 1. 一句话

装配层从"两处手写"变成**一个库**：`inc/chassis_loop.hpp`（84 行，INTERFACE 库，无 `.cpp`），
接口按 `../DESIGN.md` 冻结（D1~D9），测试 7 组 / 50 条断言，**行为与改造前逐位相同**。

## 2. 验收数字（全部实测）

| 项 | 结果 |
|---|---|
| 四档编译 | Debug ✅ · Release ✅ · 严格（`-Wconversion -Wshadow -pedantic -Wold-style-cast -Wuseless-cast`）**零告警** ✅ · 消毒（ASan+UBSan）✅ |
| 本组件测试 | 7 组 / 50 条断言 `ALL PASS` |
| 聚合 `ctest` | **9/9**（根 `CMakeLists.txt` 末尾挂 `add_subdirectory(chassis_loop)`） |
| 独立构建（CI） | `standalone` matrix 加了一格 `chassis_loop`，含"注册测试数 = 1"断言 |
| 库模式（被消费） | 0 个泄漏可执行文件 · `ctest -N` = 0 · 消费方程序跑通（手算核对 `0.3/0.03 = 10.00` ✓） |
| 判别力 | **13 个变异全部变红**（含 `cmd()`=限幅前、限幅挪到逆解后、标称 `dt`、无视 `count_`、`set_sink` 不转发…） |
| **金标（第 ⑥ 步）** | KND_Trial 的 `app.cpp` 改用本组件 → 仿真输出 **md5 逐位相同**（`9ab64f43…`），4 秒 4000 拍 |

## 3. 首轮手写时撞到的两处（都已清）

1. `chassis_.foward_kinematics(...)` 拼写错（少了 `r`）→ 编译不过。
2. 只读口命名漂移：实现叫 `wheel_cmd(i)`，契约（`../DESIGN.md` §7）是 `wheel_target(i)` → 测试编译不过。

两条教训已入 [`../AGENTS.md`](../AGENTS.md) §4 账本（"契约名冻结后实现照抄" / "内部名别跨命名家族"）。

## 4. 顺带发现的真 bug（未修，记 `TODO.md` P24）

**容量不一致 → 越界**：契约 `WheelSpeeds.values_[6]` 允许 6 轮，装配层只有 `Wheel* wheels_[4]`。
`ChassisLoop<OmniDrive>`（wn = 6）在 `wheels_[i]` 上越界 —— UBSan + ASan 实测：

```
runtime error: index 4 out of bounds for type 'Wheel *[4]'
AddressSanitizer: SEGV ... in Wheel::set_cmd(float)  ← 段错误
```

**为什么 7 组测试 + 消毒档都没抓到**：测试只用 `MecanumDrive`(4) 与 `DiffDrive`(2)，
`OmniDrive` 一次都没构造。教训同样是账本的一行：**"支持 N" 的组件必须测边界 N**。

> 现实影响面：当前两个消费方（KND_Trial 的麦轮、`fw_poc`）都是 4 轮 → 触发不到。
> 但只要有人用 5/6 轮全向底盘（`OmniDrive` 本来就支持），就是**静默越界写**。

## 5. 第 ⑥ 步的现场记录（可复现）

```bash
# 改造前
cmake -S ~/Develop/Workspace/KND_Trial/sim -B /tmp/knd_sim_base && cmake --build /tmp/knd_sim_base -j
/tmp/knd_sim_base/knd_sim > /tmp/knd_before.txt      # pose=(0.3425, 0.1085) yaw_odo=0.2574 | 四轮 0.52 1.40 1.04 0.88

# 改造后（app.cpp 只留"造装配层 + 姿态融合"两件事）
… > /tmp/knd_after.txt
diff /tmp/knd_before.txt /tmp/knd_after.txt          # 无输出 = 逐位相同
```

KND_Trial **未纳管 git** → 改前手工备份在 `/tmp/knd_backup/`（`CMakeLists.txt` / `app.hpp` / `app.cpp`）。
改动：`firmware/CMakeLists.txt` 加 `knd_add_dep(chassis_loop …)` + 链进 `knd_app`；
`app.hpp` 的手写链（`chassis_` / `limiter_` / `odom_` / 4×`Wheel` / `target_ws_` / `tick_`）全部删掉，
换成 `ChassisLoop<MecanumDrive> loop_`；`app.cpp::tick` 变成 `loop_.tick(dt, hal::tick())` + 姿态。
