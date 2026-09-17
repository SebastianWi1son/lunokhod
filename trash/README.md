---
class: log
generated: false
---
# trash/ — 待裁决区

> **这里不是"已删除"。** 放的是**判断为"非文档 / 已过期 / 重复"的内容**，原信息**一字未删**，
> 等判定后由 agent 逐条执行，再决定整目录删除。
> 建立时间：2026-09-13（第一次文档卫生整理，**源码一行未改**）。

---

## 1. 里面的东西是什么

| 路径 | 它到底是什么 | 为什么在这里 | 判定后怎么处理 |
|---|---|---|---|
| `WORK_FREEZE.md`（2026-09-17） | **D 施工单** —— 冻结前收尾（P25 限幅饱和标志 / P12 `OmniDrive` 轮数） | 两项均已闭合：P25 用户手敲已核（装配层 9/9 + 39 条红→绿）；P12 改为 **模板参数** `OmniDrive<N>` + `static_assert` + **CMake 反例编译测试**（守卫被删则配置阶段失败）；KND 仿真输出仍逐位不变 | **删** |
| `WORK_SEAM.md`（2026-09-17） | **D 施工单** —— 执行器组接缝（"通用框架"）+ 逐处对照表 | 已验收：四档零告警 · 聚合 9/9 · 12 变异全红 · **KND 仿真输出逐位相同**；契约与决策已入 `DESIGN.md` §5.2 / D11 / D12 | **删** |
| `WORK_CHASSIS_LOOP.md`（2026-09-16） | **D 施工单** —— chassis_loop（装配层）的参考实现 | 已验收：四档编译零告警 · 7 组测试 · 聚合 9/9 · 13 变异全红 · **KND_Trial 仿真输出 md5 逐位相同**；代码本体才是源 | **删**（历史在 git；契约与决策已入 `chassis_loop/docs/DESIGN.md`，验收记录入 `chassis_loop/docs/log/ACCEPTANCE.md`） |
| `WORK_ODOMETRY.md`（2026-09-14） | **D 施工单** —— odometry 的参考实现（带 D1~D10 决策点标注） | 已验收（`test_odometry` **116/116**），施工单使命结束；代码本体才是源 | **删**（历史在 git；验收记录已入 `kinematics/docs/IMPL.md` §9 + `docs/TODO.md` P15） |
| `WHEEL_BUILD_GUIDE.md`（348 行） | **D 施工单** —— control/wheel 的构建指导 | wheel 已完成并发布 v0.1.0，施工单使命结束；教训已沉淀进 `control/wheel/docs/log/WHEEL_LESSONS.md` | ① 删（历史在 git）；或② 抽出“设计决策”并入 wheel 的 B 类文档 |
| `AHRS_BUILD_GUIDE.md`（107 行） | **D 施工单（草案 v0，2026-08-22，"明天拍板"）** | AHRS 姿态解算已由**独立项目 `foucault`** 承接（见 `kinematics/docs/ODOMETRY_DESIGN.md` §11） | ① 删；或② 改成指向 foucault 的一行指针 |
| `sensor_fusion/`（README 96 行 + legacy 8 文件） | 一个**自我声明已降级**的目录 | 见下方 §2 | ① 删；或② 若还要用，压缩成 `kinematics/examples/dual_sensor_fusion.cpp` |

### 关于 `sensor_fusion/`

三件事同时成立，所以整目录进 trash：

1. **它自己说不是产品** —— `README.md` 开头："定位：**不是独立产品。** 是 Kinematics 运动学库的**宣传素材**和**备用代码模板**。"
2. **它的 `legacy/` 与 `kinematics/legacy/` 逐字节完全相同** —— `diff -rq` 无输出，2200 行纯重复（`cdd402e` 加入时叫"参考"）。仓库里不该有两份一样的 C 源码。
3. **它的 README 声称的文件不存在** —— README 说"完整示例见 `kinematics/examples/dual_sensor_fusion.cpp`"，但 `kinematics/examples/` 只有 `example.cpp` + `simulation_demo.cpp`。`kinematics/docs/DESIGN.md` 的项目结构图也这么写 → **同一个假事实写在两处**。

---

## 2. 本次整理做了什么（全部 `git mv`，可完整回退）

| 动作 | 详情 |
|---|---|
| 建 `log/` | `docs/log/`、`kinematics/docs/log/`、`control/wheel/docs/log/` |
| A 类归位 | `docs/REVIEW_RESPONSE.md` → `docs/log/` |
| A 类归位 | `docs/STAGE1_REVIEW.md`、`docs/STAGE2_REVIEW.md` → `kinematics/docs/log/` |
| A 类归位 | `control/wheel/docs/WHEEL_LESSONS.md` → `control/wheel/docs/log/` |
| 组件文档归位 | `docs/DESIGN.md`、`docs/THEORY.md`、`docs/DEV_GUIDE.md` → `kinematics/docs/` |
| 全局规则上移 | `control/wheel/docs/GIT.md` → `docs/`（内容通用，只是举例用 wheel） |
| 进 trash | `docs/WHEEL_BUILD_GUIDE.md`、`docs/AHRS_BUILD_GUIDE.md`、`sensor_fusion/` |
| 新建 | `docs/README.md`（文档体系 + 模板） |
| 新建 | `trash/README.md`（本文件） |
| 加头 | 15 个文档加上 `class:` front-matter + 类规则（**只加，未删**） |
| 改引用 | 4 个文件的路径引用（kinematics/AGENTS.md、IMPL ×2、DESIGN.md） |

**没有删除任何信息，没有修改任何源码。**

---

## 3. 诊断：为什么这个仓库的文档会乱

**根因只有一条**：根 `docs/` 是 **kinematics 独立成库时期**的文档目录。
`cdd402e chore: lunokhod 根级基线（docs 体系 + sensor_fusion legacy 参考）` 把它并进 monorepo 时，
**没有把组件专属的文档归位到组件目录**，于是根 `docs/` 变成三种东西的混装：

1. 真·全局（ARCHITECTURE / TODO / ALGO_LIB_DECISION）
2. kinematics 专属（DESIGN / THEORY / DEV_GUIDE / STAGE1 / STAGE2）
3. 其他组件专属（WHEEL_BUILD_GUIDE / AHRS_BUILD_GUIDE）

本次整理把 2、3 各归各家。**以后新文档按 §4「变更频率」判，不按"放哪儿方便"判。**

---

## 4. 待裁决清单（矛盾 / 过期 / 重复 / 缺口）

每条附证据，判定后可直接执行。

### P1 — 明确的矛盾与过期（已实证）

| # | 问题 | 证据 | 建议 |
|---|---|---|---|
| 1 | `TODO.md` **P10「根目录无 git，docs/ 不在版本控制，三仓互不关联」已过期** | 根 git 仓已存在（23 commits，起点 `cdd402e 根级基线`），`docs/` `control/` `kinematics/` 全部被追踪 | 关闭 P10（选项 a「并成一个仓」已执行） |
| 2 | `TODO.md` **P2「IMU 姿态解算系统 未开工」已过期** | 已由独立项目 `foucault` 承接（`ODOMETRY_DESIGN.md` §11 把 foucault 列为既有组件） | P2 改成一行指向 foucault |
| 3 | `kinematics/docs/DESIGN.md` §项目结构 **整块与代码不符** | 写 `include/kinematics/`、`differential_drive.hpp`、`mecanum_drive.hpp`、`speed_limiter.hpp`、`tests/`、`README.md`；实际是 `inc/`、`drive_diff.hpp`、`drive_mecanum.hpp`、`drive_omni.hpp`、`test/`，且 **kinematics/ 下没有 README.md** | 整块重写（或标注"早期草案"）。漂移清单已由 `IMPL.md` §7 记了 8 条 |
| 4 | `kinematics/docs/DESIGN.md` 首段宣称"**一个 header 拖进工程**" | 实际要 include `chassis.hpp` + 3 个 `drive_*.hpp` + `kinematics.hpp` + `contracts.hpp` 共 6 个 | 改成"一个聚合入口 `chassis.hpp`" |
| 5 | `dual_sensor_fusion.cpp` 存在于**两处文档**但**文件不存在** | `sensor_fusion/docs/README.md` + `kinematics/docs/DESIGN.md` 项目结构图；`REVIEW_RESPONSE.md` 已证实 | 随 `sensor_fusion/` 一起裁决 |
| 6 | `kinematics/docs/THEORY.md` 的 Omni 公式**符号与代码相反** | `IMPL.md` §5.3：文档 `+sin/−cos/−R`，代码 `−sin/+cos/+R` | 加醒目标注"以代码约定为准"（接线时会导致轮子反向） |
| 7 | **跨项目规则冲突**：`kinematics/AGENTS.md` 说"禁止直接修改项目代码文件"，未区分源码与测试 | `foucault` 项目 2026-08-30 已定案"**测试一律由 AI 编写**" | **需你拍板**：lunokhod 是否也适用"测试 AI 写、源码人写"？见 §5 |
| 8 | `kinematics/docs/DESIGN.md` "预期 Stars 100-250" 主观预测 | P13 已记 | 删（P13 顺手做） |

### P2 — 内容重复（同一条事实多处）

| # | 重复的事实 | 出现在 | 唯一归属应为 |
|---|---|---|---|
| 9 | legacy C 导航源码（2200 行，逐字节相同） | `kinematics/legacy/` 与 `trash/sensor_fusion/legacy/` | `kinematics/legacy/` |
| 10 | 三类底盘的正/逆运动学公式 | `kinematics/docs/DESIGN.md` §支持底盘 + `THEORY.md` §2 + `IMPL.md` §5 | 公式归 DESIGN；THEORY 只留推导与出处 |
| 11 | TwistAccLimiter 设计（STAGE 3） | `kinematics/docs/DEV_GUIDE.md` STAGE 3 + `kinematics/docs/DESIGN.md` §SpeedLimiter + `control/twist_acc_limiter/docs/IMPL.md` | 组件自己的文档 |
| 12 | "实际目录结构" | `IMPL.md` ×2 + `kinematics/AGENTS.md` 架构速览 | 一处（建议组件 README 或 IMPL） |

### P3 — 结构 / 命名缺口

| # | 问题 | 建议 |
|---|---|---|
| 13 | ~~**缺根 `README.md`**~~ —— ✅ **已建 2026-09-14**（一屏门面：组件清单 + 怎么跑 + 文档指向） | — |
| 14 | ~~**缺根 `AGENTS.md`**~~ —— ✅ **已建 2026-09-14**（全局铁律 + oracle 规则 + 文档/git 规则，一屏） | — |
| 14b | **`AGENT.md` 从来没被自动加载过** —— pi 只认 `AGENTS.md` / `CLAUDE.md` | ✅ 已全仓更名为 `AGENTS.md`（39 处引用同步） |
| 15 | **完全没有 `STATUS.md`** —— "现在能跑什么"只散在 `IMPL` 的实测段和 `TODO` | 建 `docs/STATUS.md` + 生成脚本（C 类"不许手写"） |
| 16 | `IMPL.md` ×2 —— **与代码天然重复**，规则还是"改代码必须同步本文档" | 这是持续成本最高的一项。建议**降级**：只留"一屏文件地图 + 追加式变更记录（带日期）"，不再逐行复述代码 |
| 17 | 各组件缺 `README.md`（`kinematics/` `control/wheel/` `control/twist_acc_limiter/` 都没有） | 组件自述在哪、怎么编、怎么测；上游文档只链接 |
| 18 | 分支名不统一（P4）：根仓在 `master`，`GIT.md` 教的是 `master → main` | 拍板一个方向，然后全仓统一 |
| 19 | `kinematics/src/` 是**空目录**（`IMPL.md` §2 的目录图也没列它） | 删掉，或说明为何保留 |
| 20 | ~~`kinematics/CMakeLists.txt` 没有 `enable_testing()` / `add_test`~~ | ✅ **已修 2026-09-13**：已补 `enable_testing()` + 4 个 `add_test`，`ctest` 现可真实运行 |

### P4 — 缺失机制（不是矛盾，是缺口）

| # | 缺口 | 对应规则 |
|---|---|---|
| 21 | ~~无 CI（P14）~~ —— ✅ **已建 2026-09-14**：`.github/workflows/ci.yml`（三组件 matrix + 金标可复现 job） | 批次验收自动化 |
| 22 | ~~无 LICENSE（P13）~~ —— ✅ **已加 2026-09-14**：MIT | — |
| 23 | 无 STATUS 生成脚本 | C 类"状态不许手写" |
| 24 | 无"文档路径 / 符号存在性"检查 | 防 P1-3 / P1-5 类问题复发（本次靠人工跑才找出来） |
| 25 | 无"施工单验收后移出 `docs/`"的检查 | D 类规则 |

---

## 5. 需要你拍板的两个决策点

### 决策 1：`AGENTS.md` 的"禁止改代码"包不包括测试？

- `kinematics/AGENTS.md` 现状："AI 只允许指导、讲解、复查、贴参考代码，**禁止直接修改项目代码文件**"（未区分源码 / 测试）
- `foucault` 已定案（2026-08-30）：**测试一律由 AI 编写，源码由用户手写**
- 两个项目规则不一致 → 建议统一，但由你决定方向

**如果同意统一为"测试 AI 写"**，则 odometry 的下一步立刻可执行：AI 出 `test_odometry.cpp`（9 条锚点）+ 尺子审计，你只签字。

### 决策 2：`IMPL.md` 是保留全文还是降级？

- 保留全文 = 每次改代码都要同步 150 行散文（当前规则），这是**你文档量的最大来源**
- 降级 = "一屏文件地图 + 追加式变更记录"，同步成本从"重写"降到"加一行"

---

## 6. 删除本目录的前置条件

- [ ] §1 三个条目已判定
- [ ] §4 的 P1 八条全部关闭
- [ ] §4 的 P2 四条重复已收敛到唯一归属
- [ ] §5 两个决策点已拍板
- [ ] §4 的 P4 机制已建（至少 23、24）
