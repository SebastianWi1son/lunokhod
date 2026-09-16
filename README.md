---
class: fact
generated: false
---
# lunokhod

嵌入式底盘控制系统 —— **手写重构项目**：把 2024 年的 C 语言巡线代码，逐个重构成可复用、可测试的 C++17 组件。

- 每个组件：**零依赖**、能直接拖进 STM32 工程、带行为锚点测试
- 源码由作者手写；测试由 AI 编写（出题人 / 答题人分工，见 [AGENTS.md](AGENTS.md)）

## 接手先读（零上下文入口）

| 顺序 | 读什么 | 为什么 |
|---|---|---|
| 1 | [`docs/log/HANDOFF.md`](docs/log/HANDOFF.md) | **交接快照**：当前状态、工具用法、下一步任务、已知坑 |
| 2 | [`AGENTS.md`](AGENTS.md) | 铁律（角色分工 / oracle 规则 / 库的职责边界） |
| 3 | [`docs/README.md`](docs/README.md) | 文档体系与六条硬规则 |
| 4 | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 上下行链、汇合点 |
| 5 | [`chassis_loop/docs/DESIGN.md`](chassis_loop/docs/DESIGN.md) | **装配层**的契约与决策（要做底盘编排就读它；施工单在同目录） |

## 这是什么

lunokhod 是**底盘控制系统的库集合** —— 几个互相独立、可单独取用的 C++17 库，
外加一个开发期聚合器。

- 每个库：**零依赖**（只用标准库头文件）、能单独拖进 STM32 工程
- 每个库：**可单独构建、单独测试、单独被消费**（被别的工程引入时自动进入“库模式”）
- 源码由作者手写；测试由 AI 编写（见 [AGENTS.md](AGENTS.md)）

## 库清单

| 库 | 路径 | CMake target | 依赖 | 用途 |
|---|---|---|---|---|
| **contracts** | [`contracts/`](contracts/) | `contracts` | — | 数据契约：`Twist` / `WheelSpeeds` / `Pose`（**最底层**） |
| **kinematics** | [`kinematics/`](kinematics/) | `kinematics` | `contracts` | 平面底盘运动学正/逆解（差速 / Mecanum / 全向 N 轮） |
| **odometry** | [`odometry/`](odometry/) | `odometry` | `contracts` | 轮速 → 位姿积分 + 逐拍记录（`SampleSink`） |
| **wheel** | [`control/wheel/`](control/wheel/) | `wheel` | — | 单轮执行层：S 曲线规划 → 速度环 PID → PWM |
| **twist_acc_limiter** | [`control/twist_acc_limiter/`](control/twist_acc_limiter/) | `twist_acc_limiter` | `contracts` | Twist 空间三通道加速度限幅（斜坡发生器） |
| **chassis_loop** | [`chassis_loop/`](chassis_loop/) | `chassis_loop` | 上面全部 | **装配层**：限幅 → 逆解 → N×轮控 → 正解 → 里程计（拥有唯一心跳） |

**依赖方向只能单向，且全部汇于最底层** `contracts`：

```
contracts ← kinematics           contracts ← odometry
contracts ← twist_acc_limiter    wheel（谁都不依赖）
        ⬑______ chassis_loop（在上面全部之上）______⬏
```

新增依赖前先看 [`docs/TODO.md`](docs/TODO.md) 的固件主线。

全局架构（下行命令链 + 上行感知链 + 汇合点）见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

## 三种用法

**① 开发者：一键构建 + 全部测试**

```bash
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure          # 8 个测试
```

**② 开发者：只搞一个库**

```bash
cmake -S kinematics -B build/kinematics && cmake --build build/kinematics -j
ctest --test-dir build/kinematics --output-on-failure
```

**③ 消费方（固件工程 / 别的项目）：只取需要的库**

```cmake
add_subdirectory(<lunokhod>/kinematics ${CMAKE_BINARY_DIR}/_ext/kinematics)
target_link_libraries(fw PRIVATE kinematics)
```

被引入的组件会**自动进入“库模式”**：只出库，不生成测试 / 示例 / 工具
（交叉编译固件时这一点是必须的）。可运行的完整范例见 `~/Develop/Workspace/fw_poc/`。

> ⚠️ **不要** `add_subdirectory(lunokhod)` —— 根目录是开发期聚合器，
> 引它会连带生成全部组件的测试可执行文件。想要“一键构建”用 ①。

所有 target 都开在 `-Wall -Wextra -Werror`（告警即错误）。

## 推之前先本地跑一遍 CI

```bash
scripts/ci_local.py            # 用工作区当前状态（含未提交改动），~7 秒
scripts/ci_local.py --clean    # 用 git HEAD 新建 clone —— 查“有东西忘了提交”
scripts/ci_local.py --job aggregate     # 只跑一个 job
scripts/ci_local.py --list              # 看有哪些 job
```

它解析 [`.github/workflows/ci.yml`](.github/workflows/ci.yml)，对每个 job 展开
`matrix`、跳过 `uses:` 步骤、在**独立临时工作区**里逐条执行 `run:` —— 就是 runner 干的事。

**为什么要它**：CI 跑在 GitHub 上（私有仓库拿日志要等 1~2 分钟）；本地 7 秒就有完整输出。

> 局限：不模拟 runner 镜像（本机工具链版本可能与 `ubuntu-24.04` 不同）。
> 缺命令的步骤会显示为 `⚠️ 跳过` 而非失败（如本机没装 `pip`）。

## 文档

**先读 [`docs/README.md`](docs/README.md)** —— 它说明有哪些文档、各自属于哪一类、怎么写才不烂。

| 我想知道 | 看哪 |
|---|---|
| 全局架构（两条链 + 汇合点） | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| 现在能做什么、还剩什么 | [`docs/TODO.md`](docs/TODO.md) |
| 某个组件怎么设计的 | `<组件>/docs/DESIGN.md` |
| 代码实际长什么样 | `<组件>/docs/IMPL.md` |
| git 工作流 | [`docs/GIT.md`](docs/GIT.md) |
| AI 怎么和我协作 | [`AGENTS.md`](AGENTS.md)（全局）+ [`kinematics/AGENTS.md`](kinematics/AGENTS.md)（组件级） |

文档规则一句话：**一条事实只写一处；日志只增不改；状态不手写。**

## 许可

[MIT](LICENSE)
