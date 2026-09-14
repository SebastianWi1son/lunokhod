---
class: fact
generated: false
---
# docs/README.md — lunokhod 文档体系（唯一说明）

> 新增或修改**任何**文档之前，先读这一页。
> 它回答三件事：**有哪些文档 / 各自属于哪一类 / 怎么写才不烂。**
> 建立于 2026-09-13（第一次文档卫生整理，源码一行未改）。

---

## 1. 唯一原则

> **同一条事实，只允许存在一处。**
> **分文件按【变更频率】，不按主题。**

矛盾的根因从来不是"文档太多"，而是**同一句话说两遍**。

---

## 2. 四类文档（硬边界，不许混）

| 类 | 变更频率 | 规矩 | 谁写 | 因为什么烂 |
|---|---|---|---|---|
| **A 日志** | 永不改 | **只增不改**；每条带日期；不声称"当前状态" | 人 | 不会烂（只增不改就不可能自相矛盾） |
| **B 事实** | 跟代码 / 决策变 | **唯一来源**；别处只许链接，不许复制 | 人 | 复制到第二处时烂 |
| **C 状态** | 每批 / 每次跑都变 | **不许手写**，脚本生成 | 脚本 | 手写时烂（散文没有更新触发器） |
| **D 施工单** | 一次性 | **用完即弃**，验收后进 `trash/` | AI 出稿、人照抄 | 留着就变成"第二个代码源"，必然过期 |

- 矛盾只可能出现在 **B、C** 两类。
- **同一类里可以按主题再分文件；不同类绝不能混进同一个文件。**
- A 类里"老条目和新条目说法不同"**不是矛盾**，是理解演化 —— 只要每条带日期，且不声称代表当前状态。
- **D 类是这次整理才显式命名的**：你手敲工作流的产物（构建指导、伪代码指南）既不是历史也不是事实，它是**一次性施工单**。它天然会过期，所以必须标记出来，验收后进 `trash/`。

---

## 3. 文件头模板（每个文档第一行必须是）

```markdown
---
class: log | fact | status | work
generated: true | false
---
```

- `class` —— 决定这份文件适用哪条规则。
- `generated: true` —— 表示"禁止手改"，改完会被下一次生成覆盖。

---

## 4. 事实归属表（查东西先查这张表，不要全文搜索）

| 事实 | 唯一归属 | 状态 |
|---|---|---|
| 全局架构（两条链 + 汇合点） | [`ARCHITECTURE.md`](ARCHITECTURE.md) | ✅ |
| 全局 git 工作流 | [`GIT.md`](GIT.md) | ✅ |
| 算法库策略（A1~A8） | [`ALGO_LIB_DECISION.md`](ALGO_LIB_DECISION.md) | ✅（待拍板） |
| 进度 / 待办 / 挂起事务 | [`TODO.md`](TODO.md) | ✅ |
| kinematics 设计与公式 | [`../kinematics/docs/DESIGN.md`](../kinematics/docs/DESIGN.md) | ✅（已知漂移，见其头注） |
| kinematics 理论推导 | [`../kinematics/docs/THEORY.md`](../kinematics/docs/THEORY.md) | ✅ |
| **odometry 设计**（当前施工） | [`../kinematics/docs/ODOMETRY_DESIGN.md`](../kinematics/docs/ODOMETRY_DESIGN.md) | ✅ v3 定稿 |
| 代码**实际**长什么样 | **代码本身**（`IMPL.md` 只是地图） | ⚠️ 见 §7 |
| 现在能跑什么、验收数字 | **⚠️ 缺** —— 应建 `STATUS.md` | ❌ 见 `trash/README.md` |
| AI 协作规则（组件级） | [`../kinematics/AGENTS.md`](../kinematics/AGENTS.md) | ✅ |
| AI 协作规则（**全局级**） | [`AGENTS.md`](AGENTS.md) | ✅ |
| 踩过的坑 / 阶段复盘 | `<组件>/docs/log/*.md` | ✅ |
| 外部评审查验 | [`log/REVIEW_RESPONSE.md`](log/REVIEW_RESPONSE.md) | ✅ |
| 施工单（怎么改代码） | `<组件>/docs/*_WORK.md`、`DEV_GUIDE*.md`（**临时物**） | ⚠️ 用完即弃 |

---

## 3b. 命名规则（通用 / 专案 / 一次性）

> 判据只有一条：**这个名字会被复用吗？**
> 会被复用的 → **固定名**；只此一份的 → **随意**。

### ① 通用文档 —— **固定单词名**（不带主题，看到名字就知道角色）

| 固定名 | 角色 | 放在哪 |
|---|---|---|
| `README.md` | **所在目录的门面**：这是什么 + 怎么跑 | 项目根 / 组件根 || `AGENTS.md` | AI 协作规则 | 项目根 / 组件根 || `DESIGN.md` | 设计权威（为什么这么设计） | `<组件>/docs/` |
| `ARCHITECTURE.md` | 全局架构（只有多组件项目需要） | 项目根 `docs/` |
| `TODO.md` | 待办 / 挂起事务（**唯一进度源**） | 项目根 `docs/` |
| `STATUS.md` | 当前状态（脚本生成） | 项目根 `docs/` 或组件 `docs/` |
| `GIT.md` | git 工作流 | 项目根 `docs/` |
| `THEORY.md` | 理论推导与外部参考 | `<组件>/docs/` |
| `IMPL.md` | 代码实际长什么样（代码地图） | `<组件>/docs/` |

**不加限定词。** 不写 `IMPLEMENTATION_TRUTH.md` / `THEORY_AND_REFERENCE.md` / `GIT_BRANCHING.md` ——
那是把“内容描述”当名字，越写越长。
角色用**固定词**，主题靠**位置**表达：它在谁的 `docs/` 下，就是谁的设计。

> ⚠️ **`AGENTS.md` 的 S 不是笔误** —— pi / Claude Code 的约定名就是 `AGENTS.md`（或 `CLAUDE.md`）。
> 写成 `AGENT.md` 的工具**不会自动加载**，规则会变成“写在纸上但没人看”。
> （本项目 2026-09-14 才发现这个问题，全仓 39 处引用已改名。）

### ② 专案文档 —— `<主题>_<角色>.md`

一个主题一份、会复现的文档。角色后缀**只准用这三个**：

```
_DESIGN     设计        例：ODOMETRY_DESIGN.md
_DECISION   决策        例：ALGO_LIB_DECISION.md
_WORK       施工单      例：WORK_ODOMETRY.md（一次性 → 验收后进 trash/）
_REVIEW     审查 / 复盘  例：STAGE1_REVIEW.md（一次性 → 放 log/）
```

### ③ 一次性文档 —— **名字随意**

日志、复盘、施工单、外部评审 —— 统统放 `log/`。
名字怎么写都行，**因为没人需要靠名字找它第二次**。

### ④ 本项目 2026-09-13 的改名记录

| 旧名（又长又乱） | 新名 | 依据 |
|---|---|---|
| `IMPLEMENTATION_TRUTH.md` ×2 | `IMPL.md` | 通用角色 → 固定名 |
| `THEORY_AND_REFERENCE.md` | `THEORY.md` | 同上 |
| `DEV_GUIDE_PSEUDOCODE.md` | `DEV_GUIDE.md` | 施工单，去掉“伪代码”这个实现细节 |
| `GIT_BRANCHING.md` | `GIT.md` | 通用规则 → 固定名 |
| `PENDING_ITEMS.md` | `TODO.md` | 同上（且与 "TODO" 惯例对齐） |
| `ALGO_LIB_STRATEGY.md` | `ALGO_LIB_DECISION.md` | 专案文档 → `_DECISION` 后缀 |
| `docs/README.md` | （不改） | README = **目录门面**，这是标准做法 |

---

## 5. 目录地图

```
lunokhod/
├── README.md                          B 事实：门面（是什么 + 怎么跑）
├── AGENTS.md                          B 事实：全局 AI 规则（pi 只加载 AGENTS.md / CLAUDE.md）
├── LICENSE                            MIT
├── .github/workflows/ci.yml           CI：三组件编译+测试、金标可复现
├── docs/
│   ├── README.md                      B 事实 ← 你在这里（文档体系 + 命名规则）
│   ├── ARCHITECTURE.md                B 事实：全局架构
│   ├── GIT.md                         B 事实：git 工作流
│   ├── ALGO_LIB_DECISION.md           B 事实：算法库策略（专案）
│   ├── TODO.md                        C 状态：唯一待办源
│   └── log/
│       └── REVIEW_RESPONSE.md         A 日志：外部评审查验
├── kinematics/
│   ├── AGENTS.md                       B 事实：组件级 AI 协作规则
│   ├── inc/  test/  examples/  legacy/
│   └── docs/
│       ├── DESIGN.md                  B 事实：设计权威
│       ├── THEORY.md                  B 事实：理论推导与开源参考
│       ├── ODOMETRY_DESIGN.md         B 事实：专案设计 ← 当前施工
│       ├── DEV_GUIDE.md               D 施工单：五阶段（1~3 完成）
│       ├── WORK_ODOMETRY.md           D 施工单：odometry 参考实现 ← 当前施工
│       ├── IMPL.md                    C 状态：代码地图
│       └── log/
│           ├── STAGE1_REVIEW.md       A 日志
│           └── STAGE2_REVIEW.md       A 日志
├── control/
│   ├── wheel/docs/log/
│   │   └── WHEEL_LESSONS.md           A 日志：踩坑与收获
│   └── twist_acc_limiter/docs/
│       └── IMPL.md                    C 状态：代码地图
└── trash/                             待裁决：判定后整目录删除
```

**规则**：`log/` 目录永远和它所属的文档住在一起（组件级文档配组件级 `log/`），不要把所有日志堆到根。

---

## 6. 六条硬规则

```
① 一条事实只写一处
② 日志只增不改（每条带日期，永不回头编辑）
③ 状态不手写（脚本生成）
④ 文档里禁止出现行号；禁止出现"当前状态"
⑤ 新建文档前必须回答：它的【变更频率】和现有哪个文件不同？
   答不出 → 追加到现有文件，不许新建
⑥ 改动触及事实时，必须在【同一次改动内】更新那一处
```

第 ⑤ 条是防"文档爆炸"的唯一闸门：**新建文档的合法理由只有"变更频率不同"**，不是"这是个新主题"。

---

## 7. 机器能判的，不要写成文字

按"机器能不能判"分配规则，**不要按重要性分配**：

```
能写成脚本判的        → 脚本 / CI          （最硬，不需要人记）
判不了但每次都要守的  → AGENTS.md          （常驻，≤ 一屏）
只在特定任务用的流程  → skill              （按需加载）
其余                  → 别写
```

| 规则 | 放哪 | 为什么 |
|---|---|---|
| 文档里引用的路径 / 符号必须存在 | **CI 脚本** | 机器能判（本次整理就是靠人工跑这个才找出漂移的） |
| 施工单验收后必须移出 `docs/` | **CI 脚本** | 检查 `class: work` 的文件是否还在 |
| `STATUS.md` 必须是生成的 | **CI 脚本** | 重跑无 diff |
| 命名约定 / 成员尾下划线 / 禁止 camelCase | `AGENTS.md` | 每次都要守 |
| **oracle 必须外借，不许自造；借不到要上报** | `AGENTS.md`（kinematics） | 每次写测试都要守 |
| 不许改 legacy/（只读参考） | `AGENTS.md` | 常驻约束 |
| 文档四类 + 新建文档判据 | `AGENTS.md`（3 行）+ 本页 | 每次写文档都要守 |
| **批次验收**流程 | **skill** | 特定任务，步骤多 |
| **文档对账**流程 | **skill** | 同上 |
| **尺子审计**流程 | **skill** | 同上 |

### 待建 skill（三个）

| skill | 触发 | 干什么 |
|---|---|---|
| `批次验收` | 一批代码敲完 | 三档编译 + 全量测试 + 逐条对照验收标准 + 更新 `IMPL` |
| `文档对账` | 冻结前 / 定期 | 读 B 类文档，逐条列出对代码的断言 → 到仓库核对 → 输出一致性表 → 不一致即失败 |
| `尺子审计` | 写完一批测试 | 对每条断言回答：期望值从哪来 / 是否用了被测对象 / 改坏一行会不会红 |

---

## 8. 新文档开工模板

```markdown
---
class: <log|fact|status|work>
generated: false
---
# <文件名> — <一句话定位>

> **类：<A 日志 / B 事实 / C 状态 / D 施工单>** —— <这条类规则的一句话复述>。
> <指向唯一来源的链接>

## ...

<!-- 日志类：下面只允许追加，格式为
### [YYYY-MM-DD] 标题
- 现象 / 根因 / 修复 / 教训
-->
```
