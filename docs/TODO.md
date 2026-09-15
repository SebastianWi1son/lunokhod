---
class: status
generated: false
---
> **类：C 状态** —— **全项目唯一的待办 / 挂起事务来源**。改完划掉，新增补上，带日期。

# lunokhod 悬挂事项清单（PENDING ITEMS）

> 记录所有"知道存在但没闭合"的事项。每条含：状态 / 内容 / 下一步 / 优先级。
> 更新规则：处理完一条划掉，新增一条补上，日期标注。

---

## ⭐ 第一需求：麦轮 2.5D 底层固件（2026-09-14 定）

> **目标**：结合 **foucault**（IMU 姿态）+ **lunokhod**（底盘控制），做出一套可跑的
> **麦轮（Mecanum）2.5D 底盘底层固件** —— 这是本项目的第一个真实产品目标。
>
> 状态：**需求已定，尚未拆解**。下面是盘出来的缺口，**待用户确认后立项**。

### 已有积木（可复用）

| 积木 | 位置 | 状态 |
|---|---|---|
| 麦轮运动学（正/逆解） | `kinematics/inc/drive_mecanum.hpp` | ✅ 已验收（互逆锚点全绿） |
| 里程计 | `kinematics/inc/odometry.hpp` | ✅ 已验收（140 断言） |
| Twist 限幅 | `control/twist_acc_limiter/` | ✅ v0.1.0 |
| 单轮执行（S 曲线 + PID） | `control/wheel/` | ✅ v0.1.0 |
| 6 轴姿态（Mahony） | `foucault`（独立仓） | ⚠️ PC 侧已验收，**未上过真机、无 IMU 驱动** |

### 缺口（按依赖顺序）

| # | 缺口 | 说明 | 有现成代码吗 |
|---|---|---|---|
| **G0** | **库要能被消费**（可被 add_subdirectory / FetchContent 引入） | 4 个库都缺守卫、缺可链接 target；已修，详见下方 | ✅ **已修** |
| **G1** | **平台层 / HAL 抽象** | 轮子要 `MeasureSpeedFn` / `SetPwmFn`，现在没人实现 → 需编码器定时器 + PWM 的 STM32 驱动 | ⚠️ PoC 有 PC 假实现，**真机实现未写** |
| **G2** | **IMU 驱动** | ICM-20602 → foucault 的 `IMUSample` 接口 | ⚠️ legacy 有 `Lib/driver/hal/imu/icm20602/` |
| **G3** | **融合层** | odometry 的 `yaw_ref` ↔ foucault 的 yaw 互补；打滑降权（见 P19） | ❌ 无（组件不存在） |
| **G4** | **调度与时间基** | 固定周期循环（50 Hz ~ 1 kHz），`tick` 从哪来 | ❌ 无 |
| **G5** | **通信协议** | 上层怎么下发 `Twist`、怎么回传位姿/状态 | ❌ 无 |
| **G6** | **实车标定** | 轮径 / 轮距 / `twist_scale` / IMU 安装对齐 | ❌ 无（需硬件） |
| **G7** | **安全** | 看门狗、失控保护、上电自检 | ❌ 无 |

### 已确认的需求（2026-09-14）

| 项 | 结论 |
|---|---|
| **“2.5D” 含义** | 主要在**水平地面**工作，但**存在 30° 上坡** → 平面运动学 + **倾角修正**（详见下方“倾角带来的两个后果”） |
| **MCU** | **STM32H750XBH6** （2026-09-15 更正：此前记为 H743 是错的，已作废） |
| **片内 flash** | **128 KB**（H750 是 H743 的 value-line），不是宽裕的 2 MB |
| **外扩 flash** | **W25Q64（8 MB）** —— 在核心板上（2026-09-15 补记）；需先配 QUADSPI 才能用 |
| **底盘** | **4 麦轮**（两两平行、常规四轮安装 → 与 `MecanumDrive(lx, ly, r)` 的 X 型约定一致 ✅） |
| **编码器** | **定时器正交**（STM32 TIM Encoder Mode） |
| **IMU** | **SPI**（ICM-20602） |
| **上层** | **遥控器**（→ 需要接收机协议解析层） |
| **现状** | 算法层 5 块积木已就位，**具体控制环一点没做**（G1/G4/G5 全是空地） |
| **固件工程** | 见 `KND_Trial`（**另开仓库**，不并进 lunokhod）—— 硬件实配以那里的 CubeMX 工程为准 |

### ⚠ 30° 坡带来的两个后果（**需要拍板**）

**后果 1：里程计水平位置会多报 15.5%**

车沿坡面走了 L 米，水平投影只有 `L·cos(30°) = 0.866 L`。推导：

```
车体速度 (vx, vy, 0) → 世界速度 = Rz(ψ)·Ry(θ)·Rx(φ)·(vx,vy,0)
无 roll 时展开，水平分量 = (vx·cosθ, vy) 再按 yaw ψ 旋转
                            ↑ 只有前进分量要乘 cosθ
```

**处理建议（保持解耦）**：**在平台层修正，odometry 一行不改** ——
调用方拿到 foucault 的 pitch 后，先把 `vx` 乘 `cos(pitch)` 再喂 `update()`。
理由：一旦让 odometry 自己吃姿态，它就绑死 IMU 了，破坏“底盘无关”红线。

**后果 2：坡上重力分量 = 0.5 g，车会往下溜**

速度环会自动补，但要注意：
- **积分饱和** —— 持续偏差会让 PID 积分项跑满（`limit_i_` 要调）
- **Mahony 的 pitch**：匀速上坡时加速度计只反映重力 → pitch 准；但**加减速时会混入运动加速度** → pitch 短暂出错（6 轴 IMU 的固有局限）
- **麦轮横移能力下降**：滚子在坡面上侧向力损失，坡上尽量少用 vy

### 仍待补充

1. **控制环频率**（建议 1 kHz，与 PoC 一致）与 **IMU 采样率**
2. **遥控接收机型号 / 协议**（SBUS / CRSF / iBUS / PWM？）
3. **遥控器↔Twist 的映射**：摇杆量 → 速度上限？要不要模式开关（自稳 / 手动）？
4. **IMU 具体型号确认**（ICM-20602？）

### 仓库组织决策（2026-09-14 拍板）

**结论：保持 monorepo，不拆仓库；但每个库必须先满足“可独立消费”的四条。**

| 问题 | 结论 |
|---|---|
| lunokhod 下是多个子库吗？ | **是** —— 3 个库（kinematics / wheel / twist_acc_limiter）+ 1 个**开发期聚合器** |
| 需要拆成多个仓库吗？ | **不需要**。内部只有 **1 条**依赖边（`twist_acc_limiter → kinematics`）；开发期契约（`Twist`/`WheelSpeeds`/`Pose`）还在变，跨仓改一次要约 N 个 PR + N 次发版 + 同步 pin；单人项目用不到独立发版的好处；**不拆可逆（`git subtree split` 保历史），拆了再合很麻烦** |
| 需要补什么？ | ① 可单独构建 ✅ ② 被消费时进入库模式 ✅ ③ 导出可链接 target ✅ ④ **顶层聚合器**（一键构建全部）✅ |
| 固件放哪？ | **另开仓库** —— 应用 = 消费者，依赖方向与库相反；混进来会让 lunokhod 变成“库 + 一个具体产品” |

**拆仓触发条件**（满足任一再拆）：
1. 某个库被**别的项目**独立使用，且版本节奏与 lunokhod 不同
2. 某个库大到需要独立的 CI / issue 追踪
3. 多人分守不同库

**三种用法（README 已写，CI 三个 job 逐一守护）：**

```
① 开发者一键  cmake -S . -B build && ctest            → 8/8
② 单库开发    cmake -S kinematics -B build && ctest     → 4/4
③ 被消费      add_subdirectory(<lunokhod>/kinematics)   → 只出库，零泄漏
```

### 盘出来的技术前提（2026-09-14 已修）

| # | 坑 | 修法 |
|---|---|---|
| **G0-a** | 4 个库都**没有 `PROJECT_IS_TOP_LEVEL` 守卫** → 被固件工程引入时会连着生成一堆宿主可执行文件 | 已给 4 个库都加上守卫：作为子项目时只出库 |
| **G0-b** | `twist_acc_limiter` 无条件 `add_subdirectory(kinematics)` → 固件同时引两者时 **target 重名报错** | 改成 `if(NOT TARGET kinematics)`，优先用消费方已引入的 |
| **G0-c** | `foucault_core` 是 INTERFACE 库，**不含 `mahony.cpp`** → 外部工程链接不到符号 | 新增 `foucault_solver`（STATIC），导出完整可链接的 target |

### PoC（已跑通）

`~/Develop/Workspace/fw_poc/` —— 4 个库拼成一条链，PC 上跑通：

```
遥控 ──► Twist ──► 限幅 ──► 逆解 ──► 4×轮子(曲线+PID) ──► PWM ──► 假电机
                                              │
                                              ▼ 编码器
       姿态(foucault) ◄── 融合 ◄── 里程计 ◄── 正解
```

结果：`yaw_odo=0.2574` 与 `yaw_imu=0.2521` 收敛到一起 → **融合闭环真的在工作**。
同时也暴露了：PID 参数是编的，电机响应很慢 —— 那正是**实车标定（G6）**要解决的事。

---

## 主线（底盘运动系统）

### P1. LineFollower（灰度寻迹）⏸️ **暂时搁置（2026-09-14）**
- **内容**：控制链最后一块——灰度传感器 → 偏差 → 纠偏 Twist。完成后 `灰度→纠偏→限幅→逆解→轮速` 全闭环
- **素材**：legacy C 巡线导航（`Lib/control/nav/`）
- **下一步**：先读 legacy 巡线代码，定组件边界（传感抽象 + 纠偏控制器 + Twist 输出）
- **优先级**：🔴 高 → ⏸️ **搁置**（第一需求变更为「麦轮 2.5D 底层固件」，见上方新主线）

### P2. IMU 姿态解算系统 ⬜ 设计阶段
- **内容**：小车 IMU 需求（先 gyro_z，扩展多维姿态解算）。ICM-20602 = **6 轴无磁力计**（3 陀螺 + 3 加速度计）→ yaw 只能纯陀螺积分（漂移），roll/pitch 靠加速度计互补
- **方向**：最小通用核心（镜像 wheel 思路）——`Ahrs` 组件：gyro 积分 + accel 互补（Mahony）+ 可选 mag；Config 聚合 + dt 穿透 + 单位约定 + 行为锚点
- **素材**：`Lib/driver/hal/imu/icm20602/`（驱动）+ 用户 vibecode 过的 Mahony
- **下一步**：先讲原理（互补滤波/四元数/Mahony），再定组件边界写 DESIGN
- **优先级**：🔴 高（近期有真机需求）

## 架构/一致性

### P3. Ramp 跨项目统一（DRY）⬜ 待决策
- **内容**：`control/wheel/src/ramp.cpp` 与 `twist_acc_limiter` 的 `ramp()` **算法完全相同**（max_step=rate·dt + clamp）
- **权衡**：提取公共 `inc/ramp.hpp`（两项目共用）vs 接受重复 20 行（零依赖原则）
- **下一步**：决策后执行；注意 wheel v0.1.0 已发布，提取是 v0.2.0 的变更（破坏性）
- **优先级**：🟡 中（两项目都已发布，随时可做）

### P4. 分支名统一 ✅ **已完成（2026-09-14）**
- **结果**：两个仓库（lunokhod / foucault）**本地与远端全部统一为 `main`**，旧 `master` 已删除
- **过程**：远端 URL 从 HTTPS 改成 SSH（HTTPS 没配凭证助手）→ 改默认分支 → `push main` → 删 `master`
- **教训**：改默认分支必须在网页操作（`PATCH /repos/...` 需 token）；
  不先改默认分支直接删会被拒：`refusing to delete the current branch`

### P5. kinematics docs 去留 ⬜ 待拍板
- **内容**：(a) 仓库保留副本（开源库惯例）vs (b) 瘦身纯代码库（README 指路上级目录）
- **优先级**：🟢 低

### P6. ARCHITECTURE.md 术语同步 ⬜ 部分完成
- **内容**：SpeedLimiter → TwistAccLimiter、三级愿景 → v1 范围 + Wheel 承接 S 曲线
- **优先级**：🟢 低（多数已由 agent 生成 docs 覆盖，待一致性核对）

## 重构线

### P7. FOC 重构 ⬜ 未开工
- **内容**：`Lib/foc/`（foc.c + foc_transform.h + gimbal_ctrl + motor_param + motor_tune）→ C++ 组件化
- **优先级**：🟡 中（foc 云台是用户中期目标）

### P8. Gimbal 重构 ⬜ 未开工
- **内容**：`legacy/gimbal_2axis/`（STM32H743 工程）→ 与 lunokhod 体系整合
- **优先级**：🟡 中

## 学习线（ROS）

### P9. ROS2 Humble（WSL2）⬜ 未起步
- **内容**：里程碑 1 = 发布/订阅 + kinematics 包装成 `/cmd_vel` 节点；未来 Gazebo 重仿真（可能双系统）
- **下一步**：WSL2 装 Ubuntu 22.04 + ROS2 Humble
- **优先级**：🟢 低（独立学习线，与竞赛主线并行）

## 评审收尾（来自 REVIEW_RESPONSE.md 2026-08-22 查验）

### P10. 根级版本控制决策 ⬜ 待决策
- **内容**：根目录无 git，docs/（ARCHITECTURE/DESIGN/复盘）不在版本控制；三仓互不关联 + 工作树脏（kinematics 5 docs 未提交删除、twist_acc_limiter 未提交改动）——评审最大隐患
- **选项**：(a) 并成一个仓（带根提交）vs (b) 维持三仓 + 根级 docs 仓
- **下一步**：决策后执行；此前先做 P11 的工作树收尾（未提交改动落盘）
- **优先级**：🔴 高

### P11. PID 默认语义决策 ⬜ 待决策（与定案冲突）
- **内容**：评审建议 `limit_out_=0` 默认改为"不限制"；但 wheel 已定案 **"0=disabled"惯例**（max_rate=0 关斜坡、thresh=0 关分离）——两者冲突
- **决策点**：0 语义维持（文档写明坑）vs 改 sentinel vs 分层（0=中立 + preset）
- **影响**：若改，牵动 wheel v0.2.0（已发布 v0.1.0 不可改）
- **优先级**：🔴 高（真实踩坑风险，但改法需先拍板）

### P12. 防御校验 ⬜ 未开工
- **内容**：OmniDrive 构造 `wn>6` 越界写 `J[6][3]`、`wn<2` 数学无意义——加断言/参数校验；twist_acc_limiter `reset()` 补测试
- **优先级**：🟡 中（2 小时量级）

### P13. LICENSE + 主观预测 ✅ **已完成（2026-09-14）**
- LICENSE（MIT）已加；`kinematics/docs/DESIGN.md` 的“预期规模”整节（含 Stars 预测）已删
- **剔除理由**（已写入设计文档）：B 类事实文档里**不许有主观预测** —— 预测必然过期，且会被后来的读者当真。
  被删内容里“\<300 行核心代码 / 5-6 个头文件”已被实测证伪（实际 1071 行 / 7 个）

### P14. CI 流水线 ⬜ 未开工
- **内容**：GitHub Actions——三仓各一（-Werror + 跑测试 + example），消除"全绿依赖手动"
- **优先级**：🟢 低（半天量级）

---

## 处理记录
- 2026-08-22：wheel v0.1.0 发布（主线第 3/4 块完成）；本文档创建
- 2026-08-22：外部评审查验（REVIEW_RESPONSE.md 落盘，14 条全属实无误报）；新增 P10~P14；docs 落盘（WHEEL_LESSONS / TODO / AHRS_BUILD_GUIDE / REVIEW_RESPONSE）
- 2026-09-13：文档卫生整理（分类/移动/trash）+ odometry 金标与测试落地；新增 P15~P18
- 2026-09-14：**odometry 落地**（P15 关闭，140/140）；文档命名统一（7 个改名）；错误账本建立；
  AGENTS.md 改名（从此能被自动加载）；LICENSE / CI / 根 README / 根 AGENTS 补全；P13、P17、P18 关闭；
  新增 P19（打滑检测，未立项）

---

## 2026-09-13 新增（odometry 开工前的尺子审计所得）

### P15. 手写 `inc/odometry.hpp` ✅ **已完成（2026-09-14）**
- **结果**：`test_odometry` **140/140**，`ctest` **4/4 Passed**，`-Wall -Wextra -Werror` 零告警
- **首次手敲的 3 个坑**（契约字段名不符 / `vy*sin` 写成 `vy*cos` / 同类型字段聚合初始化静默错位）
  **全部被测试抓到** → 规则已入 `kinematics/AGENTS.md` 错误账本
- **收尾**：施工单 `WORK_ODOMETRY.md` 已进 `trash/`；代码地图已追加到 `kinematics/docs/IMPL.md` §9

### P16. `Odometry` 缺初始位姿入口 ⬜ 待拍板（R12）
- **内容**：`reset()` 只清零，没有 `reset(const Pose&)` / `set_pose()`。
  设计 §8-#5 “初始 yaw=π/2” 无法直接表达（测试已用“先纯转再走”绕开）
- **权衡**：加 `reset(const Pose&)`（一行，代价低）vs v1 坚持“恒从原点起”（但上电时就地开里程计就做不到了）
- **优先级**：🟡 中（一行代码，但影响接口契约）

### P17. 两处 2D 位姿积分的积分顺序不一致 ✅ **已统一（2026-09-14）**
- **结论**：`examples/simulation_demo.cpp` 已改为**半隐式欧拉**（先积分 yaw，再用新 yaw 旋转位移），与 odometry R6 同源
- **重要发现**：改动后 **完整输出（含每段终点 + 轨迹图）逐字符相同** ——
  因为 demo 的路径**闭合且对称**，显式的 +φ/2 滞后与半隐式的 −φ/2 超前成对抵消。
  所以统一是**源码同源**问题，不是数值问题
- **顺带更正**：`ODOMETRY_DESIGN.md` §4 原写“半隐式比显式**少一阶误差**”是错的（已用 R14 更正：两者误差**幅值恒等**）

### P18. CI ⬜ → ✅ **已完成（2026-09-14）**
- `.github/workflows/ci.yml`：① 三组件 matrix 编译 + `ctest` ② **金标可复现** job（重跑生成器 → `git diff --exit-code`）
- 三个组件的 `CMakeLists.txt` 都补了 `enable_testing()` + `add_test`（此前 `ctest` 全是空的）

### P19. 打滑检测（Slip Detection）⬜ **未立项**（2026-09-14 提出）
- **现状**：**不在计划里**。只有 `kinematics/docs/ODOMETRY_DESIGN.md` §10「未来拓展」记了一笔。
  `odometry` 出数据但**不做判定**（红线：判定需要 IMU 当参照，一旦进来就从“底盘无关”变成“绑死 IMU 硬件”）
- **三个层级**（复杂度 / 可信度递增）：
  - **L0**（只要编码器）：`cmd` 期望 vs `twist` 实际，滑动窗口内比值持续低 → 报“疑似打滑 / 堵转”
  - **L1**（+陀螺，抓**单边打滑**，最实用）：残差 `twist.wz·dt − gyro_z·dt` 超阈值
  - **L2**（+加速度计）：短时加速度积分位移 vs 里程计位移
- **前置条件（关键）**：这是**异常检测**，阀值不能手拍 —— 需要“正常”与“打滑”的**标注数据**，现在两份都没有
- **建议路径**：先在 **PC 侧**做（odometry sink 导 CSV + foucault replay 导 IMU，时间戳对齐 → Python 里试算法 / 定阈值），
  验证过再考虑落到嵌入式融合层
- **归属**：未来的**融合层**组件（可挂 foucault estimator），**不是** odometry
- **底盘相关性分析（2026-09-14）**：三层检查里 **L0 / L1 / L2 全部底盘无关** ——
  因为 odometry 的出口是 `Twist`，底盘差异在 forward kinematics 那一步已经被消掉了；
  **只有「轮间一致性」是底盘相关的**（麦轮 4 轮 vs 差速 2 轮 vs 全向 3 轮，残差定义不同）。
  → 所以可插拔的正确切法是：**检测器保持底盘无关，底盘差异用 `ChassisProfile`
  （`holonomic` / `wheel_count`）参数化**，而「轮间残差」由 `<Chassis>` 侧算好喂进来。
  **不是 Policy 模板** —— 变化的只有参数，不是流程。
- **优先级**：🟡 中（被“先采数据”卡住）
