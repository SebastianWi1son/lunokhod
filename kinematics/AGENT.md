# AGENT.md — lunokhod/kinematics 项目代理指南

## 项目定位

C++17 header-only 零依赖嵌入式底盘运动学库（差速/Mecanum/全向 + SpeedLimiter）。
- 源码在 `inc/`（用户唯一入口 `kinematics.hpp`）
- 架构/公式：`docs/DESIGN.md`（权威）；开发流程：`docs/DEV_GUIDE_PSEUDOCODE.md`
- **阶段复盘（含错误清单+正确示例+讨论收获）：`docs/STAGE1_REVIEW.md`、`docs/STAGE2_REVIEW.md`**——新阶段开工/复查代码前必读
- 参考实现：`legacy/`（2024 年 C 语言 nav 模块）

## 重要：这是学习项目

- 项目作者是 C++ 学习者，正在**古法编程**（不依赖 AI 独立写代码）
- **AI 只允许指导、讲解、复查、贴参考代码，禁止直接修改项目代码文件**
- 所有建议必须解释"为什么"，给出客观标准，让作者自己动手改

## 架构速览（阶段 2 已完成：diff + mecanum + omni + 翻译官）

```
inc/contracts.hpp          数据层：Twist（输入契约）、WheelSpeeds（输出契约），POD struct
inc/kinematics.hpp     数学核心：Kinematics<Derived>（CRTP 能力接口）+ jacobian_apply 翻译官 + k2PI
inc/drive_diff.hpp     DiffDrive : public Kinematics<DiffDrive>
inc/drive_mecanum.hpp  MecanumDrive（4×3 J，forward 伪逆）
inc/drive_omni.hpp     OmniDrive（N×3 J，forward 3 轮特例 γ=0）
inc/chassis.hpp        对外聚合入口（用户只 include 这一个）
test/test_kinematics.cpp   测试驱动（断言 + 退出码，三底盘全绿）
examples/                  使用演示（独立 main）
```

- 命名语义：**chassis（应用外壳）在上，kinematics（数学）在下**；具体底盘（DiffDrive 等）才是"底盘"，基类是"运动学能力接口"（Comparable 模式）
- CRTP 派发：基类 const 方法 `static_cast<const Derived*>(this)->xxx_impl()`
- 接口层/实现层方法均须 `const`（纯函数，不持有状态；唯一有状态的是 SpeedLimiter（可选附赠，不在 chassis.hpp 聚合入口））
- jacobian_apply：`template<uint8_t N>` 绑定数组尺寸，运行时行数 wn（diff=2，mec=4，omni=wn_）；forward 目前手写（pinv 展开），不抽矩阵
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

## 测试与验证

- 测试退出码 = 结果（0 = 全过），浮点断言必须用容差（1e-5）不能 `==`
- 模板未实例化 = 未真正编译：任何改动必须用 test 实际调用验证
- 编译命令：`g++ -std=c++17 -Wall -Wextra -Iinc test/test_kinematics.cpp`
