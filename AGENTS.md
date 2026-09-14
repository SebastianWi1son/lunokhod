---
class: fact
generated: false
---
# AGENTS.md — lunokhod 全局代理规则

> **类：B 事实（规则）**。只放【每个组件都要守】的约束，压在一屏内。
> 组件特有的规则在 `<组件>/AGENTS.md`（目前只有 [`kinematics/AGENTS.md`](kinematics/AGENTS.md)）。
> 文档体系与命名规则见 [`docs/README.md`](docs/README.md)。

## 1. 项目一句话

lunokhod = 嵌入式底盘控制系统。把 2024 年的 C 语言巡线代码，逐个重构成**零依赖、可拖进 STM32 工程**的 C++17 组件。

## 2. 铁律（不可违反）

1. **源码默认只读** —— AI 不得修改任何组件的 `inc/` `src/` `examples/`，除非用户明确指示。
   默认允许：`test/`、`docs/`、`CMakeLists.txt`、`.github/`、新建文件。
2. **测试由 AI 写、源码由用户手写** —— AI 是出题人，用户是答题人（2026-09-13 定案）。
3. **oracle 必须外借，不许自造** —— 见 §4。
4. **可回退** —— 覆盖 / 删除 / 移动前必须可回退（已提交 / 备份）。三者皆无 = 禁止执行。
5. **不静默吞错** —— 脏数据（NaN / Inf / 越界）必须传播或可检出，不得静默洗成合法值。
6. **文档与代码不一致 = 缺陷** —— 发现即记录（进 `trash/` 或 `docs/TODO.md`），不许只放在嘴上。

## 3. 命名

- `snake_case`；**成员统一尾下划线 `_`**；**参数与局部变量不带 `_`**；bool 成员 = `is_` 前缀。
- 禁止 camelCase；领域术语优先（`twist` / `odometry` / `kinematics` / `residual`）。
- 名字不绑定单位与实现细节。

## 4. oracle 规则（金标不许自造）

测试里每个"期望值"都必须能回答：**它从哪来？** 借的优先级（能用上面的就不许往下走）：

```
① 真值     —— 实测真值 / 第三方库（numpy·scipy·sympy）/ 论文例题 / 参考实现
② 性质     —— 群性质·互逆·往返一致·线性/不变性·守恒·极限·收敛阶
③ 独立复算 —— 另一条数学路径的闭式解（例：等比级数 ↔ SE(2) 指数映射）
④ 独立实现 —— 用另一种语言 / 工具重算同一件事
```

**借不到就上报，不许自己拍一个数。** 三条硬约束：

1. 期望值**不得**由被测对象计算（反例：用 `rotate()` 造 `rotate()` 的输入）
2. 测试的**输入**也不得由被测对象生成
3. 容差必须**推导**（实测累积误差 × 安全系数），不许手拍 `1e-5`

金标生成器必须**自检**：闭式 vs 数值循环 vs 第三方库三方对比，并在输出里打印结果。

## 5. 工程

- C++17；库**零依赖**（仅 `<cmath>` / `<cstdint>`）；MCU 侧 `-fno-exceptions -fno-rtti`。
- 告警全开 `-Wall -Wextra -Werror`（**告警即错误**）；改完必过编译 + 全量 `ctest`。
- 声明与定义分离（header-only 组件例外）；常量与参数聚合进 `Config`。

### 5.1 库的硬约束（做库而不是做应用）

每个组件都必须满足下面四条 —— 这是“它能被别人单独取用”的全部含义：

```
① 可单独构建      cmake -S <组件目录> -B build && ctest
② 可被当子项目引入  add_subdirectory(<组件目录> ...) —— 不污染消费方
③ 导出可链接 target target_link_libraries(x PRIVATE <库名>) 就够了
④ 不反向依赖      依赖方向只能单向：kinematics ← twist_acc_limiter ← …
```

具体做法（已在 4 个库里落地，新库照抄）：

```cmake
# ② 构建模式守卫：被引入时只出库
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    return()
endif()

# ③ 导出可链接的 target（有 .cpp 就必须进 STATIC 库，不能只给头文件路径）
```

> 反面教材（2026-09-14 修掉的三个真坑）：4 个库都没有守卫（固件会被迫
> 连测试一起编）· `twist_acc_limiter` 无条件 `add_subdirectory(kinematics)`
> （同时引两者就 target 重名）· `foucault_core` 是 INTERFACE 库不含 `mahony.cpp`
> （外部链接不到符号）。

### 5.2 库的仓库组织

**当前：monorepo（一个 lunokhod 装全部库）+ 每个子目录可独立消费。**

为什么 **不**拆成多个仓库：

- 内部只有**一条**依赖边（`twist_acc_limiter → kinematics`），拆仓收益极小
- 开发期契约还在变（`Twist` / `WheelSpeeds` / `Pose`）——
  跨仓改一次契约要开 N 个 PR、N 次发版、再同步 pin，**开发期这个开销远大于收益**
- 单人项目用不到 polyrepo 的好处（独立发版 / 独立 issue）
- **不拆是可逆的（随时能拆，`git subtree split` 保留历史），拆了再合很麻烦**

**什么时候该拆**（触发条件，满足任一再拆）：

1. 某个库被**别的项目**独立使用，且版本节奏与 lunokhod 不同
2. 某个库大到需要独立的 CI / issue 追踪
3. 团队多人分守不同库

**仓库边界规则**：

- `lunokhod` = **库集合**（面向“别人也能拿去用”）
- `foucault` = 独立库（姿态解算，与底盘无关，分得开）
- **固件 = 应用（消费者），另开仓库** —— 它的依赖方向与库相反，
  混进来会让 lunokhod 变成“库 + 一个具体产品”

## 6. 沟通

- 全程中文；**先讲原理**再动手；**决策点前置**（列清单 + 推荐 + 理由）。
- 面向用户的结论**统一放在回答末尾**，不与过程叙述穿插。
- 外部意见（评审 / 网文 / 其他 agent）**先验证再采纳**，逐条给结论，不盲信也不护短。
- 用户说"等等 / 停下 / 不需要"→ 立即停止。

## 7. 文档

- 四类：**A 日志（只增不改）/ B 事实（唯一来源）/ C 状态（不许手写）/ D 施工单（用完即弃）**。
- 命名：通用文档用**固定单词名**；专案文档 `<主题>_<角色>.md`。详见 [`docs/README.md`](docs/README.md) §3b。
- **新建文档前先问**："它的变更频率和现有哪个文件不同？" 答不出 → 追加，不新建。
- **文档里禁止出现行号**，只写符号名。

## 8. git

- **commit 由用户主导** —— AI 默认不 commit；消息用用户看得懂的中文。
- **push 前先在本地跑一遍 CI**：`scripts/ci_local.py`（约 7 秒）。
  它解析 `.github/workflows/ci.yml` 并把每个 job 逐条真跑 ——
  CI 在 GitHub 上，拿日志要等 1~2 分钟，本地先过一遍能省掉绝大多数来回。
  加了新 link/新依赖时，另跑 `--clean`（用 git HEAD 新建 clone，查“忘了提交”）。
- 任何 git 操作前先 `git status`；**`git add` 不会清空已有暂存区** ——
  曾经的坑：`git mv` 是立即入暂存区的，随后 `git add <几个文件>` 会把重命名一并带上。
- 文件移动用 `git mv`。
- 分支与发布流程见 [`docs/GIT.md`](docs/GIT.md)。

## 9. 错误账本

> 规则：**每踩一次坑加一行**。加行的标准是"**不写下来下次还会踩**"。只增不改。

- [`kinematics/AGENTS.md`](kinematics/AGENTS.md) —— 已有 3 条（同类型字段静默错位 / 契约名两边各写各的 / 旋转矩阵需要 vy≠0 且 yaw≠0 的用例）
- 其他组件：待建
