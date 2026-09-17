---
class: status
generated: false
---
> **类：C 状态** —— **跟代码变**。⚠️ 与代码天然重复，目标形态是「一屏文件地图 + 追加式变更记录」。
> 文档体系与写作规则：../../docs/README.md
> **命名空间（2026-09-17）**：本文档里的类型名**省略 `lunokhod::` 前缀** —— 例：`MecanumDrive` 即
> `lunokhod::kinematics::MecanumDrive`，`Twist` 即 `lunokhod::Twist`（规则见 [`AGENTS.md`](../../AGENTS.md) §3.1）。

# Odometry 实现真相文档（以代码为准）

> 生成日期：2026-09-15（本节内容自 `kinematics/docs/IMPL.md` §9 **原样迁入** —— 2026-09-15 卫生整理：
> odometry 从 `kinematics` 独立成组件，它的代码地图跟着它走）
> 本文档描述 `odometry/` 目录下**实际存在的代码**，一切以源码为准。
> 维护规则：**改代码必须同步改本文档**；本文档与代码冲突时，以代码为准并当场修正文档。

---

## 1. 项目定位

C++17 header-only 零依赖的**里程计**组件：把轮速（经 kinematics 正解得到的本体系 `Twist`）积分成
车体位姿 `(x, y, yaw)`，同时是**数据记录源**（每帧原始数据 + 位姿快照）。

**依赖只有 `contracts`**（`Twist` / `WheelSpeeds` / `Pose`）—— 与运动的**正/逆解**无关：
那一层是 `kinematics` 的活，本组件只吃它算好的瞬时速度、对它做**时间积分**。

**明确不做**：融合算法本身（IMU + 里程计融合属 foucault / 融合层）· 打滑**检测算法**
（只保证数据形状够用）· 路径规划 / 轨迹生成 · 轮径在线标定（只留 `twist_scale_` 修正口）。

## 2. 实际目录结构

```
odometry/
├── inc/
│   └── odometry.hpp            ← 里程计：有状态积分器 + 可选记录（INTERFACE 库对外产物）
├── test/
│   ├── test_odometry.cpp       ← 里程计测试（140 断言，oracle 全部外部）
│   ├── odometry_golden.hpp     ← ⚠ 自动生成，勿手改（gen_odometry_golden.py 产物）
│   └── tools/gen_odometry_golden.py  ← 金标生成器（scipy / sympy / numpy 交叉验证）
├── tools/
│   ├── odometry_demo.cpp       ← 四段场景 → run.csv
│   └── plot_odometry.py        ← CSV → 四格图
├── docs/
│   ├── ODOMETRY_DESIGN.md      ← B 事实：专案设计（v3 定稿）
│   ├── IMPL.md                 ← 你在这里（C 状态：代码地图）
│   └── log/ODOMETRY_FAQ.md     ← A 日志：概念答疑录
├── CMakeLists.txt
└── AGENTS.md                   ← 代理协作规则（inc/ 人写、test/ AI 写）
```

## 3. 接口（5 个公开成员）

| 成员 | 语义 |
|---|---|
| `Odometry(float twist_scale = 1.0f)` | 唯一构造参数 = 标定系数（**硬件属性**） |
| `set_sink(SampleSink, void* ctx)` | 注册数据出口；nullptr = 不记录 |
| `reset()` / `reset(const Pose&)` | **只清位姿**；`twist_scale_` / `sink_` **保留**（清状态，不清配置）。R12：带参重载 = 设到指定位姿 |
| `update(twist, dt, tick, cmd, ws) → Pose` | 每帧一次；返回值与 `pose()` 同源 |
| `pose()` / `yaw_continuous()` / `yaw_ref()` | wrap / 连续 / 连续（融合层校准源） |

## 4. 参数分两类（本组件最容易读错的地方）

| 类别 | 字段 |
|---|---|
| **参与计算** | `body_twist`、`dt` |
| **只作记录** | `tick`、`cmd`、`ws` |

## 5. 行为要点

- **积分**：半隐式欧拉（R6）—— 先 `yaw_ += wz·dt`，再用**新** yaw 的 `cos/sin` 旋转位移。
- **标定**：`twist_scale_` 乘 **vx / vy / wz 三个通道**（漏 `wz` → 每圈丢约 11°）。
- **记录**：`ws == nullptr` 时**整帧不记**（R7）；样本里 `twist_` 是**标定后**的值，`cmd_` **原样**。
- **yaw**：内部只存**一个连续值**；两个出口各取所需（连续 vs wrap），**不存第二个成员**。

## 6. 测试与金标（oracle 全部外部）

| 文件 | 作用 |
|---|---|
| `test/test_odometry.cpp` | **140 断言，11 组**（金标 / 世界系旋转 / 收敛 / 零输入冻结 / reset / wrap / 单步 / scale 不变性 / sink / R7 防御 / **初始位姿（R12）**） |
| `test/tools/gen_odometry_golden.py` | 金标生成器（**五重自检**，打印闭式 vs 循环 vs scipy 的偏差） |
| `test/odometry_golden.hpp` | ⚠ 自动生成，勿手改 |

**oracle 来源**：等比级数闭式（离散精确解）+ SE(2) 指数映射（连续真解，scipy DOP853 交叉验证）+ `numpy.angle`（wrap）+ **实测推导的容差**。

## 7. 构建与测试（实测）

- CMake ≥ 3.28，C++17。`add_library(odometry INTERFACE)`（header-only），暴露 `inc/`，`INTERFACE` 依赖 `contracts`。
- 两种构建模式：**开发模式**（单独构建 / 根聚合器）生成 `test_odometry` + `odometry_demo`；
  **库模式**（被固件等消费方引入）只出库。
- 全部 target 启用 `-Wall -Wextra -Werror`。
- **2026-09-15 实测（独立成组件后）**：聚合构建 `ctest` → **8/8 Passed**（含 `test_odometry`）；
  单库构建 `cmake -S odometry` → **1/1 Passed**；`test_odometry` → **140/140 断言**，零告警。

## 8. 验收记录

- 手写者：作者本人（AI 未改 `inc/`）；测试由 AI 编写。
- 首次手敲出现 3 个坑（契约字段名 / `vy*sin` / 同类型字段静默错位），**全部被测试抓到**；规则已入 `AGENTS.md` 错误账本。

## 9. 开发 / 分析工具

| 文件 | 作用 |
|---|---|
| `tools/odometry_demo.cpp` | 四段场景（直行 / 圆弧 / 麦轮横移（注入 40% 打滑）/ 原地转）→ 导 `run.csv`（850 帧 × 15 列） |
| `tools/plot_odometry.py` | `run.csv` → 四格图（轨迹 / yaw 出口 / 打滑信号 / 半隐式 vs 显式放大对比） |

```bash
cmake -S odometry -B build && cmake --build build -j
./build/odometry_demo run.csv
python3 odometry/tools/plot_odometry.py run.csv run.png
```

> 这两个是**工具**，不是库示例 —— 它们依赖 `inc/odometry.hpp`，但库本体不依赖它们。
> **用途**：CSV 是打滑检测 / 底盘建模 / 复盘分析的原料（见 `../../docs/TODO.md` P19），
> 也是将来实车数据回放与 PC 侧算法验证的入口。

## 10. 已知未闭合

见 [`ODOMETRY_DESIGN.md`](ODOMETRY_DESIGN.md) §7b：
**R12**（无初始位姿入口 —— 待拍板是否加 `reset(const Pose&)`）、
**R8~R11**（尺子审计提出的 4 条坏尺子修正）。
