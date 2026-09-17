---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。 组件级 AI 协作规则。
> 文档体系与写作规则：../docs/README.md

# AGENTS.md — lunokhod/kinematics 项目代理指南

> 2026-09-15：里程计（odometry）与数据契约（contracts）已从本组件**独立成库**：
> 契约 → [`../contracts/`](../contracts/)；里程计 → [`../odometry/`](../odometry/)。
> 下面关于它们的表述已相应修正，相关错误账本与代码地图也已随对象迁走。

## 项目定位

C++17 header-only 零依赖嵌入式底盘运动学库（差速/Mecanum/全向 + SpeedLimiter）。
- 源码在 `inc/`（**瞬时映射唯一入口 `chassis.hpp`**）
- 架构/公式：`docs/DESIGN.md`（权威）；开发流程：`docs/DEV_GUIDE.md`
- **阶段复盘（含错误清单+正确示例+讨论收获）：`docs/log/STAGE1_REVIEW.md`、`docs/log/STAGE2_REVIEW.md`**——新阶段开工/复查代码前必读
- 实现真相（以代码为准）：`docs/IMPL.md`（改代码必须同步该文档）
- 参考实现：`legacy/`（2024 年 C 语言 nav 模块）

## 重要：这是学习项目

角色按**文件**划分，不按“重要程度”划分：

| 目录 | 谁写 | 说明 |
|---|---|---|
| `inc/` `src/` `examples/` | **作者手写** | AI 禁止直接修改；只给伪代码/参考/复查 |
| `test/` | **AI 写**（2026-09-13 定案）| AI 是出题人，作者是答题人；见下方 oracle 规则 |
| `docs/` | AI 可写 | 按 `../../docs/README.md` 的四类规则 |

- 项目作者是 C++ 学习者，正在**古法编程**（不依赖 AI 独立写代码）
- 所有建议必须解释“为什么”，给出客观标准，让作者自己动手改

## 架构速览（阶段 2 已完成：diff + mecanum + omni + 翻译官）

```
inc/kinematics.hpp     数学核心：Kinematics<Derived>（CRTP 能力接口）+ jacobian_apply 翻译官 + k2PI
inc/drive_diff.hpp     DiffDrive : public Kinematics<DiffDrive>
inc/drive_mecanum.hpp  MecanumDrive（4×3 J，forward 伪逆）
inc/drive_omni.hpp     OmniDrive<N>（N×3 J，forward 通用 N 轮伪逆，任意 γ；**轮数是模板参数**）
inc/chassis.hpp        对外聚合入口（瞬时映射唯一入口，用户只 include 这一个）
test/test_kinematics.cpp   测试驱动（断言 + 退出码，三底盘全绿）
examples/                  使用演示（独立 main）
```

> **数据契约不在本组件**：`Twist` / `WheelSpeeds` / `Pose` 在 [`../contracts/`](../contracts/)。
> 本库的 `inc/` 只有上面 5 个文件。

- 命名语义：**chassis（应用外壳）在上，kinematics（数学）在下**；具体底盘（DiffDrive 等）才是"底盘"，基类是"运动学能力接口"（Comparable 模式）
- CRTP 派发：基类 const 方法 `static_cast<const Derived*>(this)->xxx_impl()`
- 接口层/实现层方法均须 `const`（纯函数，不持有状态；唯一有状态的是 SpeedLimiter（可选附赠，不在 chassis.hpp 聚合入口））
- jacobian_apply：`template<uint8_t N>` 绑定数组尺寸，运行时行数 wn（diff=2，mec=4；omni 已改成模板参数 N，无运行期轮数）；forward 目前手写（pinv 展开），不抽矩阵
- 零堆、零异常、仅 `<cmath>`/`<cstdint>`；POD 值语义（聚合初始化、可 memcpy）

## 命名评估框架（暗线：每次出现新变量/函数名时按此评估）

```
① 信息增量：后缀/前缀必须添加新信息，否则删除
   （反例：inverse_calc —— calc 无增量；正例：inverse_kinematics，IK 是领域术语）
② 领域术语优先：优先机器人/数学标准术语（IK/FK/odometry），再考虑自造词
③ 一层一个后缀：接口层无后缀，实现层 _impl，禁止混搭多套后缀体系
```

历史结论：`inverse_calc`/`forward_calc` 被评估为冗余（成员函数名本身是动词，calc 零信息增量），
建议 `inverse_kinematics`/`forward_kinematics` 或 `inverse`/`forward`。

新结论（2026-08）：`types.hpp` → `contracts.hpp`（POD 契约表意）；否决 `kcontract`/`k_contract`
（缩写前缀与库内全名风格不一致，k 还有常量/千的惯例歧义）；候选 `kinematics_types`（一致性）
vs `contracts`（表意）中选 contracts。SpeedLimiter 上报机制选方案 a（`LimitResult` 返回式，
每通道饱和标志，无事后查询时序陷阱）。

## 错误账本（每踩一坑加一行）

> **这不是章程，是账本。每一行都对应一次真实踩过的坑。**
> 加行的标准：**这个坑如果不写下来，下次还会踩。** 只增不改。

| 日期 | 坑 | 加的规则 |
|---|---|---|
| 2026-09-17 | **把「常量」当「参数」→ 定义域问题缠了三轮**：`OmniDrive(uint8_t wn, …)` 把机械轮数做成**运行期参数**，而它要写进定长 `J[6][3]` —— 于是必须回答「合法范围是多少、越界怎么办」（`wn=7` 实测**越界写栈**：`stack smashing detected`）。修法第一轮选「夹住」→ 被判定为**洗白数据**（7 轮的车被静默按 6 轮建模）；第二轮改「判无效」（`is_valid()` + 零输出）→ 仍要**调用方自觉查**，而「用户会不会遵守」的答案是**不会**；第三轮才到位：轮数是**每台车一个、构建期已知的机械事实** → 提成**模板参数**，非法轮数**写不出来** | ① **能用常量就别用参数**：把常量做成运行期参数，就必须额外回答「定义域 / 越界怎么办」，而这两件事本身就是新 bug 的来源；② 凡是「必须做的检查」，**别指望文档与自觉** —— 要么进类型系统（编译期），要么进 CI（本例新增**反例编译测试**：非法值若编得过，`cmake` 配置阶段直接失败）；③ 机械事实（轮数 / 轴距）在构建期已知，**运行期化之前先问一句「它真的会变吗」** |

> 2026-09-15：原有 3 条全部是 **odometry** 的坑（`OdometrySample` 字段顺序 / 契约字段名 /
> 旋转矩阵 `vy*sin`），已随该组件迁至 [`../odometry/AGENTS.md`](../odometry/AGENTS.md) §4。
> 本组件自己的账本是空的 —— **等第一次踩坑**。

## 测试与验证

- 测试退出码 = 结果（0 = 全过）；浮点断言必须用容差，禁止 `==`（除“零输入冻结”这类**故意**的逐位断言）
- 模板未实例化 = 未真正编译：任何改动必须用 test 实际调用验证
- 编译：`ctest --test-dir build --output-on-failure`（已注册）
  或单编：`g++ -std=c++17 -Wall -Wextra -Werror -Iinc -I../contracts/inc test/test_kinematics.cpp`

### oracle 规则（金标不许自造）

测试里每一个“期望值”都必须能回答：**它从哪来？**

借的优先级（从上往下，能用上面的就不许往下走）：

```
① 真值     —— 实测真值 / 第三方库（numpy・scipy・sympy）/ 论文例题 / 参考实现
② 性质     —— 群性质・互逆・往返一致・线性/不变性・守恒・极限・收敛阶
③ 独立复算 —— 另一条数学路径的闭式解（例：等比级数 ↔ SE(2) 指数映射）
④ 独立实现 —— 用另一种语言/工具重算同一件事
```

**借不到就上报，不许自己拍一个数。**

三条硬约束：

1. 期望值**不得**由被测对象计算（反例：用 `rotate()` 造 `rotate()` 的测试输入 —— foucault 踩过）
2. 测试的**输入**也不得由被测对象生成
3. 容差必须**推导**出来（实测累积误差 × 安全系数），不许手拍 `1e-5`

**金标生成器必须自检**：闭式 vs 数值循环 vs 第三方库三方对比，并在文件里打印结果。
范例：`../odometry/test/tools/gen_odometry_golden.py`（五重自检，含“改坏格式能不能被区分”的判别力证明）—— 它随 odometry 迁走了，但仍是全仓最好的范例。
