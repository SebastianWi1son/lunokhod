---
class: log
generated: false
---
> **类：A 日志** —— **只增不改**：每条带日期，永不回头编辑老条目，**不代表当前状态**。
> 文档体系与写作规则：../README.md

# 外部评审查验报告（REVIEW RESPONSE）

> 日期：2026-08-22
> 来源：另一 agent 对 lunokhod 的负面评价（用户截取）
> 方法：逐条实证查验（工作树 + HEAD 双视角），给出属实性判定与证据

## 一、逐条验证结果

### 1. 版本控制结构（评价：中高，最大隐患）→ ✅ 属实

| 子项 | 验证 | 证据 |
|---|---|---|
| 根目录无 git | ✅ | `lunokhod/` 无 `.git`；docs/（ARCHITECTURE/DESIGN/复盘）不在任何版本控制 |
| 三仓互不关联 | ✅ | kinematics / control/twist_acc_limiter / control/wheel 各自独立 |
| kinematics 工作树脏 | ✅ | **5 个**（评价说 4 个）docs 文件未提交删除（DESIGN/DEV_GUIDE/STAGE1/STAGE2/THEORY）+ `M CMakeLists.txt` + `M inc/drive_omni.hpp` + `M inc/kinematics.hpp` |
| twist_acc_limiter 工作树脏 | ✅ | `AM .gitignore` + `M test/...` + `?? docs/` |
| 风险结论 | ✅ | 工作树与提交不一致，未提交改动确实可能丢 |

### 2. 文档-代码漂移（评价：中，自己定的规矩）→ ✅ 属实，且比评价更微妙

| 子项 | 验证 | 证据 |
|---|---|---|
| P1 jacobian 循环 | ✅ 漂移 | **代码已修复**：`kinematics.hpp:24` 循环到 `wn`（运行时）；**文档未更**：IMPL.md §8-P1 仍写"循环上界用 N…建议改为 i<wn" |
| P2 omni forward | ✅ 漂移 | **代码已修复**：`drive_omni.hpp:23-42` 已是通用 N 轮伪逆（循环 wn_、含 gamma_）；**文档未更**：§8-P2 仍写"硬编码 3 轮 γ=0" |
| dual_sensor_fusion.cpp | ✅ 属实 | `kinematics/examples/` 只有 example.cpp + simulation_demo.cpp；README 声称的完整示例文件**不存在**（该 README 自定位"宣传素材"，但文件缺失是事实） |
| DESIGN.md omni 符号 | ✅ 属实 | DESIGN.md 写 `ω = sin·Vx − cos·Vy`，代码 `J[i][0]=−sin/…, J[i][1]=cos/…`——**符号相反**且文档未回改 |

### 3. PID 默认配置静默失效（评价：中）→ ✅ 属实

- `PIDConfig` 默认 `limit_out_=0`、`limit_i_=0` → `constrainf(x, 0)` 恒 0
- 用户只调 kp 忘设 limit → **输出恒 0 且无提示，车不动**
- 测试里每个用例显式设 `limit_out_=1e6` 恰好印证
- 本项目 wheel 的 WHEEL_LESSONS 也记录过同一坑（"默认 0 会悄悄钳死输出"）

### 4. 缺防御性检查（评价：低）→ ✅ 属实

- `OmniDrive(uint8_t wn, …)` 无校验：`J[6][3]` 固定声明，`wn>6` 越界写、`wn<2` 数学无意义
- twist_acc_limiter `reset()` 无测试（评价原文，未深查，可信）

### 5. 其他（评价：低）→ 部分属实，一处基于旧状态

| 子项 | 验证 | 证据 |
|---|---|---|
| 无 LICENSE | ✅ | 全仓无 LICENSE 文件 |
| drive_mecanum 疑问注释 | ✅ | `drive_mecanum.hpp:27` 有"// ws_fb.values_[]具体哪个对哪个呢" |
| 无 CI | ✅ | 无 .github/CI 配置，全绿依赖手动 |
| simulation_demo 无 CMake target | ⚠️ **基于已提交版属实，工作树已修** | HEAD 版 CMakeLists 无该 target；**工作树已加**（含 `-Werror`），但未提交——评价看的可能是旧状态，但其"未提交=随时丢"担忧成立 |
| DESIGN.md "预期 Stars 100-250" | ✅ | `lunokhod/docs/DESIGN.md:141` 确有此主观预测 |

## 二、评价的出入（查验修正）

1. **文档删除数量**：评价说 4 个，实际 **5 个**
2. **simulation_demo target**：评价"P3 属实"基于已提交 HEAD；工作树已补 target + 加 -Werror（未提交）
3. **P1/P2 方向判断正确**：评价说"代码已修复、文档没更新"——**验证成立**，但更准确的说法是：代码在某个时间点被修了，而 IMPL.md（号称"以代码为准的真相"）没有跟着回改，这本身就是自定的规矩被破坏

## 三、行动建议（按性价比排序）

| # | 动作 | 成本 | 对应 PENDING |
|---|---|---|---|
| 1 | 真相文档回改：IMPL.md P1/P2 标记"已修复"、P3 标记"已加 target" | 10 分钟 | P6 术语同步（扩） |
| 2 | 工作树收尾：kinematics + twist_acc_limiter 未提交改动补 commit（docs 移出是既定决策，正式落盘） | 10 分钟 | P5 docs 去留 |
| 3 | 根级版本控制决策：并仓（推荐）vs 维持三仓 + 根级 docs 仓 | 半天 | 新增 P10 |
| 4 | PID 默认语义：0=不限制 vs sentinel——**与 wheel 已定案的"0=disabled"惯例冲突**，需先决策再改（可能影响 wheel v0.2.0） | 1 天含决策 | 新增 P11 |
| 5 | 防御校验：OmniDrive 构造断言（wn∈[2,6]）+ twist_acc_limiter reset() 测试 | 2 小时 | 新增 P12 |
| 6 | LICENSE（选 MIT）+ 删 DESIGN.md 主观预测 | 10 分钟 | 新增 P13 |
| 7 | CI：GitHub Actions 三仓各一个（-Werror + 跑测试） | 半天 | 新增 P14 |

## 四、总评呼应

> "会写代码、会做数学、更会记录"的项目，欠的是"工程收尾"。
> 查验结论：**成立**。核心数学/架构评价（A−）不参与争议；工程完备度（B−）的扣分点全部有实证，其中版本控制（无根 git + 工作树脏）与 PID 默认值是真实风险，其余为收尾成本很低的打磨项。

## 五、一句话沉淀

> 第三方评审的价值在于"照镜子"：它抓到的漂移（P1/P2）、静默坑（PID limit=0）、工程洞（无根 git/LICENSE/CI）全部可复现——没有一条是误报；出入仅在数量与"基于旧状态"的细节。
