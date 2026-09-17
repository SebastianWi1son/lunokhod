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

---

## 6. 同日修正：容量 4 → 6（P24）

§4 记的那个越界当天就修了：**容量对齐契约 = 6**（`w0_..w5_` + `Wheel* wheels_[6]`），
决策记入 `../DESIGN.md` §8.2 **D10**，"未选模板容量 / 真 N 泛化"的理由也在那儿。

**修完的复验**（全部实测）：

| 项 | 结果 |
|---|---|
| 原始越界复现用例（`ChassisLoop<OmniDrive>` wn=6，ASan+UBSan） | **exit 0**（修前：`SEGV`） |
| 四档编译 | 零告警；消毒档 `ALL PASS` |
| 测试 | 8 组 / 57 条断言（新增**边界 N**：三轮 wn=3、六轮 wn=6） |
| 判别力 | 9 个变异仍全红；**另加一条"把容量退回 4" → 新用例在 ASan 下必然变红** |
| 聚合 | 9/9 |
| **KND_Trial 逐位回归** | **仍逐位相同**（容量变更对 4 轮无影响，实测确认） |

> 顺带：这条新用例第一版**自己写错了** —— 测试的假 IO `g_meas[4]` 装不下 id=5，
> 于是 ASan 先抓了测试自己的越界。**尺子也会越界**：容量改动要连测试夹具一起对齐。

---

## 7. 承接上游 ctlkit v0.1.1（2026-09-17）

算法原语（pid / lpf / ramp / smooth_planner）改为 **vendor 上游库 `ctlkit`**（`third_party/ctlkit/`，
v0.1.1，上游 sha `db2d952`），下游保留转发头与全局名。装配层侧的影响与核对：

| 项 | 结果 |
|---|---|
| 接缝设计是否受影响 | **不受影响** —— `WheelSetConfig` 照样只放一个 `PIDConfig`（新 API 具名链式设置器照样用） |
| **外仓消费方 KND_Trial** | ❌ **编译不过** —— KND_Trial 的 `app.hpp` 用旧扁平字段（`p.kd_ = …`），而 v0.1.1 已挪进 `gains_/limits_/tunings_`。**用户决定先不管下游 → 本轮不修**（补丁已备好，见下） |
| 行为锚点 | ✅ **复现**（在 KND 的 **/tmp 隔离副本**上验的，没动真工程）：`pose=(0.3425, 0.1085) yaw_odo=0.2574` ｜ 四轮 `0.52 1.40 1.04 0.88` |
| 接缝版是否仍逐位相同 | ✅ **是**（md5 `9ab64f43…`，与 ctlkit 迁移前的锚点一致） |

**修改补丁（未应用）**：`app.hpp` 里那 10 行 `[](){...}()` 换成一处具名链式调用即可（数值一个不动）——
补丁留在 `/tmp/knd_mypatch.diff`（临时目录会被清；内容就 6 行，重写也快）。

**教训**：`KND_Trial` 是**库外**消费方（且尚未提交任何 commit），**不在任何 CI 里** ——
上游破坏性改动在它身上暴露不了，只能靠人工跑一次。→ 已记 `../../docs/TODO.md` **P30**。

---

## 8. 执行器组接缝（通用框架）落地验收（2026-09-17）

接缝 = `ActuatorSet`（契约 [`../DESIGN.md`](../DESIGN.md) §5.2 · 决策 **D11**）。
用户手敲 `inc/wheel_set.hpp`（新）+ 重写 `inc/chassis_loop.hpp`；AI 负责测试、文档、验收与善后。

| 项 | 结果 |
|---|---|
| 四档编译 | Debug ✅ · Release ✅ · 严格（`-Wconversion -Wshadow -pedantic`）**零告警** ✅ · ASan+UBSan ✅ |
| 本组件测试 | **9 组**（新增第 9 组「接缝一致性」：假执行器组 `StubSet` 证明模板参数**真被用**，非装饰）`ALL PASS` |
| 聚合 `ctest` | **9/9** |
| 独立构建 / 消费模式 | `standalone` 6 组合 ✅ · 库模式零泄漏 ✅（CI 的消费者示例已切到接缝版 API） |
| 判别力审计 | **12 个变异全部变红**（含新契约的 M14「`measure` 收错设定值」→ 红 8 条）。⚠ M3（轮控用标称 `dt`）第一次**被编译器拦住**（`dt` 变未使用 → `-Werror=unused-parameter`），改用 `dt * 0.0f + 0.001f` 后变红 1 条 |
| **行为锚点（KND）** | ✅ **逐位相同** —— md5 `9ab64f43…`，与 ctlkit 迁移前、接缝改造前**完全一致**（在隔离副本上验；真工程按用户决定不动） |

**本批唯一的设计取舍（D12）**：`measure` 收设定值 —— 与 `apply` 对称，
且 `WheelSet` **除轮子外无状态**（删掉 `count_`）。修过程中顺手抓到的两处：
① 用户初稿 `measure(ws_target_)` 与契约 `measure()` 不一致（签名不匹配）；
② `: actuators_() {}` 把注入的执行器组丢掉了 → 已修（`actuators_(actuators)`）。

**文档同步**：`DESIGN.md`（§3 职责 / §5.2 契约 / §6 六步图 / §8.2 D12 / §9 F1）·
`IMPL.md`（代码地图）· 施工单 `WORK_SEAM.md`（已归档 `trash/`；§3 两段代码 = 已落地代码逐字）· CI 消费示例。
