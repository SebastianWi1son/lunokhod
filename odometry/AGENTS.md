---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。 组件级 AI 协作规则。
> 全局规则（铁律 / 命名 / oracle / 工程 / git）：[`../AGENTS.md`](../AGENTS.md)
> 文档体系与写作规则：[`../docs/README.md`](../docs/README.md)

# AGENTS.md — lunokhod/odometry 项目代理指南

> 2026-09-15：本组件从 `kinematics` 独立成库。本文件与 `docs/IMPL.md` 的内容均自
> `kinematics/AGENTS.md` / `kinematics/docs/IMPL.md` **原样迁入**，未新增也未删减信息。

## 1. 项目定位

**里程计**：把轮速（kinematics 正解得到的本体系 `Twist`）积分成车体位姿 `(x, y, yaw)`，
同时是**数据记录源**（每帧原始数据 + 位姿快照）。

- 源码：`inc/odometry.hpp`（**唯一对外产物**，INTERFACE 库）
- 依赖：**只有 `contracts`**（`Twist` / `WheelSpeeds` / `Pose`）
- 设计权威：[`docs/ODOMETRY_DESIGN.md`](docs/ODOMETRY_DESIGN.md)（v3 定稿）
- 代码地图：[`docs/IMPL.md`](docs/IMPL.md)
- 概念答疑：[`docs/log/ODOMETRY_FAQ.md`](docs/log/ODOMETRY_FAQ.md)

## 2. 定位与边界（为什么它是独立组件）

**它不属于 `kinematics`**：后者的对外身份是「**纯函数瞬时映射**」（`Twist` ⇄ `WheelSpeeds`），
而本组件是**有状态、可选、非所有用户都需要**的。

同样，**它也不属于限幅器 / 轮控**：那些是控制环，本组件是**感知侧**。

**明确不做**（v1）：

- 融合算法本身（IMU + 里程计融合是 foucault / 融合层的职责 —— 本组件只**提供**校准参考）
- 打滑**检测算法**（v1 只保证**数据形状**足够检测用；算法留 v2）
- 路径规划 / 轨迹生成
- 轮径在线标定（只留 `twist_scale_` 修正口）

## 3. 角色分工

与 `kinematics` 相同（`inc/` 作者手写、AI 禁改；`test/` AI 写）——
见 [`../AGENTS.md`](../AGENTS.md) §2 铁律。

## 4. 错误账本（每踩一坑加一行）

> **这不是章程，是账本。每一行都对应一次真实踩过的坑。**
> 加行的标准：**这个坑如果不写下来，下次还会踩。** 只增不改。
>
> 编号对齐根 [`../AGENTS.md`](../AGENTS.md) §9 的账本索引。

| 日期 | 坑 | 加的规则 |
|---|---|---|
| 2026-09-13 | **同类型相邻字段 + 聚合初始化 = 静默错误**：`OdometrySample` 里 `Twist cmd_; Twist twist_;` 顺序写反，初始化按设计顺序写 → 两个值对调，**编译器一声不吭**（同类型按位置匹配）。数学测试全绿，只有 `test_odometry` [9] 抓住 | 新写 struct 时：**相邻同类型字段要么顺序严格对齐契约，要么加一条"谁装进谁"的测试**。C++17 无 designated initializer，编译器帮不了你 |
| 2026-09-13 | **字段名与契约两边各写各的**：代码里叫 `twist_cmd_`、设计 §5 叫 `cmd_` → 测试编译不过 | **字段名是契约**。契约的源在 `docs/*_DESIGN.md`；测试从契约生成。改契约先改设计，再同步测试与代码 |
| 2026-09-13 | yaw=0 时 `vy*sin` 与 `vy*cos` 表现相同（0 vs vy），直行测试看不出来 | 写旋转矩阵时，**必须有一条 vy≠0 且 yaw≠0 的用例**（已在 `test_odometry` [1][2] 覆盖） |
|  |  |  |

## 5. 测试与验证

- 测试退出码 = 结果（0 = 全过）；浮点断言必须用容差，禁止 `==`
  （除"零输入冻结"这类**故意**的逐位断言）
- **oracle 规则**：见 [`../AGENTS.md`](../AGENTS.md) §4。
  范例就是本组件的 [`test/tools/gen_odometry_golden.py`](test/tools/gen_odometry_golden.py)
  （五重自检，含"改坏格式能不能被区分"的判别力证明）。
- 编译：`ctest --test-dir build --output-on-failure`
  或单编：`g++ -std=c++17 -Wall -Wextra -Werror -Iinc -I../contracts/inc test/test_odometry.cpp`
