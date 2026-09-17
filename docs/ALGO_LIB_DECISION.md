---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。 决策清单（A1~A8 待拍板）。
> 文档体系与写作规则：README.md

# 算法库策略 — 决策清单

> 日期：2026-08-23 ｜ 状态：**已落地（2026-09-17）** —— 决策清单 A1~A8 的逐条结果见下方「落地结果」
> 动因（用户原话）："常复用的 pid,dsp,ff 等算法打包成算法库上传 GitHub，实际开发按需 pull，
> 过程中发现 pull 下来的代码可修改升级，push 回去同步所有用到的项目。
> 猜想手敲的算法库多少会有可升级的地方，但未来多个项目不好同步。"
> 现状：lunokhod 五件套（contracts / kinematics / odometry / twist_acc_limiter / wheel）
> 同一仓内各自独立消费（2026-09-15：原先写着“三件套…各自独立仓库”，那是当时的说法）；
> **Ramp 算法在 wheel 与 twist_acc_limiter 各有一份**（PENDING P3 已标记的 DRY 问题）

## 现状盘点（哪些算法已有、在哪里）

| 算法 | wheel | twist_acc_limiter | kinematics | 备注 |
|---|---|---|---|---|
| PID（工业级） | ✅ vendor 引用（转发头） | — | — | 微分先行/梯形积分/积分分离/抗饱和/斜坡 |
| LPF | ✅ vendor 引用（转发头） | — | — | alpha = dt/(Tf+dt) |
| Ramp | ✅ vendor 引用（转发头） | ⚠️ inline 实现（数学相同） | — | **仓内 DRY 未收**（见 A7） |
| SmoothPlanner | ✅ vendor 引用（转发头） | — | — | Ramp + 两级 LPF |
| 前馈 ff | — | — | — | 待确认内容 |
| 运动学 | — | — | ✅ kinematics | 是否入库？ |

---

## 落地结果（2026-09-17）

> 上游库 = [ctlkit](https://github.com/SebastianWi1son/ctlkit)（公开 MIT，namespace `ctl`，v0.1.0 已冻结 API）。
> vendor 落在 `third_party/ctlkit/`（`inc` + `src` + `VERSION` 记来源 sha 与校验命令）。

| # | 决策点 | 落地结果 |
|---|---|---|
| **A1** | 库名 | **ctlkit · namespace `ctl`**（`ctl::PID` / `ctl::LPF` / `ctl::Ramp` / `ctl::SmoothPlanner` / `ctl::Deadzone`） |
| **A2** | 范围 | pid + lpf + ramp + smooth_planner + deadzone；**ff 未纳入**（内容待明确） |
| **A3** | 引用方式 | **b 文件复制 vendor**（原推荐 a submodule）—— 每仓 `third_party/ctlkit/` 逐字拷贝 + 旧位置留转发头（带 `// ctlkit-forwarder` 标记），配上游校验脚本守门（命令见 `VERSION`）。未用 submodule：上游发布节奏未定，且嵌入式/离线要能随仓走 |
| **A4** | 版本策略 | SemVer + 冻结分支：`v0.1.0` 冻结（minor 只增不改，breaking → major）；未发布增量走 dev 分支，冻结时再 merge 回 main |
| **A5** | wheel 的迁移方式 | **b 剪切移动**（原推荐 a 复制起步）：4 个原语副本（pid/lpf/ramp/smooth_planner）已删除，改由 vendor 提供；对消费方是**破坏性变更**（配置字段由平铺变分组路径 `cfg.limits_.limit_out_`，写法推荐具名链式 `PIDConfig{}.kp(1.0f).limit_out(1e6f)`）→ wheel 下次发布应体现 |
| **A6** | 同步机制 | 同步脚本（vendor 模式）：上游 `downstream_diff.py` —— vendor 定点逐字比对 + 转发头识别 + **未标记的同名副本一律拦下**；`--selftest` 7 例自证 |
| **A7** | twist_acc_limiter 的 ramp | ⬜ **未做**：仍是自己的 inline ramp（数学相同）。要做就换 `ctl::Ramp`，属该组件自身的破坏性变更，待其发布节奏 |
| **A8** | 测试 | 上游：oracle 黄金向量 19 例 + smoke + 判别力实测（改坏必红）；下游：保留自己的锚点测试（wheel 9 项，迁移后逐字节复现） |

---

## 决策点（当初的选项与推荐，存档）

### A1. 库名
- 选项：a) `algo-lib` b) `lunokhod-algo` c) 中性领域名（如 `control-algo`）
- 影响：GitHub 仓库名、namespace、文档

### A2. 范围（ff 是什么？）
- 选项：
  - a) pid + dsp（lpf/ramp/smooth_planner）+ ff 全打包
  - b) 只 pid + dsp（ff 内容确认后再加）
- **需用户确认：ff = feedforward 前馈组件（速度/转矩前馈）？还是别的？**

### A3. 引用方式（关键决策，决定未来同步模式）
- 选项：
  - a) **git submodule**：多项目引用同一 commit 指针，升级 = 更新指针；项目间天然同步
  - b) **文件复制 vendor**：每项目拷贝一份源码，升级 = 重新复制；"pull 下来改完 push 回"需手动同步
  - c) CMake FetchContent（网络拉取，嵌入式离线场景不友好）
- 用户描述"按需 pull / push 回 / 同步所有项目"倾向 b 的操作手感，但 a 同步更自动
- 推荐：a（submodule）+ 若未来痛则升级

### A4. 版本策略
- 选项：a) SemVer + tag（沿用 wheel 经验）b) 无版本（滚动最新）
- 推荐：a（wheel 已实战过 SemVer + 渐进稳定分支）

### A5. 现有 wheel dsp/pid 的迁移方式（wheel v0.1.0 已发布！）
- 选项：
  - a) **复制起步**：算法库从 wheel 现有代码复制为 v0.1.0，wheel 仓库保留副本（不破坏已发布仓库），后续 wheel 逐步切换到算法库引用
  - b) 剪切移动：wheel v0.2.0 移除 dsp/pid 改为引用算法库（破坏性变更，需走发布流程）
  - 推荐：a（先并行走，不阻塞 wheel；切换是后续 v0.2.0 决策）

### A6. 同步机制
- 选项：a) submodule 自动（指针更新即可）b) 同步脚本（vendor 模式）
- 影响：与 A3 联动

### A7. twist_acc_limiter 的 ramp 统一
- 选项：a) 算法库 v1 落地时顺手统一（twist 换用库 Ramp）b) 暂不动
- 影响：twist_acc_limiter 是否有破坏性变更

### A8. 测试
- 选项：沿用 wheel 锚点测试模式（每算法自带 test target + -Werror）
- 影响：算法库质量守门

---

## 与 FOC 的关系

- ✅ **已完成**（2026-09-17）：cyclotron/foc 先接入（vendor + 转发头，行为逐字节复现），
  lunokhod 随后接入（本文件所记）—— 两份活副本从此归一，上游唯一

## 明确不做

- 运动学入算法库（kinematics 是领域库非通用算法）——除非 A2 拍板包含
- 波形/调试协议层（motor_param/tune 类）——属固件层
