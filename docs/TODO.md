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
| 里程计 | `odometry/inc/odometry.hpp` | ✅ 已验收（140 断言） |
| Twist 限幅 | `command/twist_acc_limiter/` | ✅ v0.1.0 |
| 单轮执行（S 曲线 + PID） | `actuator/wheel/` | ✅ v0.1.0 |
| 6 轴姿态（Mahony） | `foucault`（独立仓） | ⚠️ PC 侧已验收，**未上过真机、无 IMU 驱动** |

### 缺口（按依赖顺序）

| # | 缺口 | 说明 | 有现成代码吗 |
|---|---|---|---|
| **G0** | **库要能被消费**（可被 add_subdirectory / FetchContent 引入） | 4 个库都缺守卫、缺可链接 target；已修，详见下方 | ✅ **已修** |
| **G1** | **平台层 / HAL 抽象** | 轮子要 `MeasureSpeedFn` / `SetPwmFn`，现在没人实现 → 需编码器定时器 + PWM 的 STM32 驱动 | ⚠️ PoC 有 PC 假实现，**真机实现未写** |
| **G2** | **IMU 驱动** | ICM-20602 → foucault 的 `IMUSample` 接口 | ⚠️ legacy 有 `Lib/driver/hal/imu/icm20602/` |
| **G3** | **残差监测 → 融合层** | 上游链缺两层（见 `../docs/ARCHITECTURE.md` §3）：① **残差监测**（两个独立来源对一下，是 FDI，**不是融合**）② **融合/状态估计**（用残差调权重）③ 修正 | ❌ 无（两层都不存在） |
| **G4** | **调度与时间基** | 固定周期循环（50 Hz ~ 1 kHz），`tick` 从哪来 | ✅ 已固化：`chassis_loop` 的 `tick(dt, now)` 就是它（见 P20）；PC 侧验证见 `KND_Trial/sim/` |
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

> **2026-09-17 退役**：`~/Develop/Workspace/fw_poc/` 的内容已全部并入 `KND_Trial`（`app/` + `sim/`），
> 两份手写链的仿真输出 **md5 逐位一致**；输出锚点已入仓 `chassis_loop/test/golden/knd_sim_anchor.txt`。
> 下面记的是它当时的样子（历史）。

`~/Develop/Workspace/fw_poc/src/main.cpp` —— 4 个库拼成一条链，PC 上跑通：

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

### P2. IMU 姿态解算系统 ⬜ **归属已修正（2026-09-15）**
- ⚠ **本条的"方向"描述的就是 `foucault`（另一仓：`~/Develop/Workspace/foucault/`） 这个库** ——
  "gyro 积分 + accel 互补（Mahony）+ 可选 mag、Config 聚合 + dt 穿透 + 行为锚点"
  与 foucault 的定位逐句对应。
- **所以它不该挂在 lunokhod 的待办里** —— 它已完成，且完成在另一个仓。
- **剩下真正未做的**：ICM-20602 的 **SPI 驱动**（→ 属固件缺口 G2，不属本仓）。
- **原始内容（存档，勿重复开工）**：内容 = 小车 IMU 需求（先 gyro_z，扩展多维姿态解算），
  ICM-20602 = 6 轴无磁力计；素材 = `Lib/driver/hal/imu/icm20602/`（驱动）+ 用户 vibecode 过的 Mahony。
- **优先级**：⚪ 库侧无需再做；驱动侧并入固件 G2

## 架构/一致性

### P3. Ramp 跨项目统一（DRY）🟡 跨仓部分已收敛（2026-09-17）· 仓内未做
- **内容**：`actuator/wheel/src/ramp.cpp` 与 `twist_acc_limiter` 的 `ramp()` **算法完全相同**（max_step=rate·dt + clamp）
- **权衡**：提取公共 `inc/ramp.hpp`（两项目共用）vs 接受重复 20 行（零依赖原则）
- **下一步**：决策后执行；注意 wheel v0.1.0 已发布，提取是 v0.2.0 的变更（破坏性）
- **2026-09-16 扩大**：同类问题已扩散到**跨仓** —— PID 有**两份行为相同的活副本**
  （`lunokhod/actuator/wheel/inc/pid.hpp` ↔ `cyclotron/foc/inc/foc/algo/pid.hpp`；`pid.cpp` 逐字节相同，
  只差 namespace / include）+ 一份**只读归档快照**（`~/Develop/Workspace/pid/source/`）
  + 需求登记在**第四处**（仓外 `~/Develop/Workspace/pid` 的 `optimization_considerations.md`，本轮新增 D3）。
  **已定约定**：PID 算法需求**归口 `~/Develop/Workspace/pid`**（登记），**落地同步两份活副本**
  （以 cyclotron 为准 —— 其 README 声明）。**待决策**：收敛成单一来源（子模块 / 单一副本 + 转发头）
  vs 接受双份人工同步。
- **2026-09-17 收敛（跨仓部分 ✅）**：PID/LPF/Ramp/SmoothPlanner 归口上游库 **ctlkit**（`third_party/ctlkit/` vendor +
  转发头 + 校验脚本）。lunokhod/actuator/wheel 与 cyclotron/foc 都改成引用上游 → **双活副本问题终结**；
  上游 spec 是行为契约唯一来源（两侧仓内文档只留链接）。
- **仍待做（仓内部分 ⬜）**：twist_acc_limiter 的 inline `ramp()` 换用上游 `ctl::Ramp`（数学相同，
  属该组件自身的破坏性变更，跟它的发布节奏走）；决策见 `docs/ALGO_LIB_DECISION.md` 的 A7
- **优先级**：🟡 中（跨仓部分已收敛；仓内那一处随时可做）

### P4. 分支名统一 ✅ **已完成（2026-09-14）**
- **结果**：两个仓库（lunokhod / foucault）**本地与远端全部统一为 `main`**，旧 `master` 已删除
- **过程**：远端 URL 从 HTTPS 改成 SSH（HTTPS 没配凭证助手）→ 改默认分支 → `push main` → 删 `master`
- **教训**：改默认分支必须在网页操作（`PATCH /repos/...` 需 token）；
  不先改默认分支直接删会被拒：`refusing to delete the current branch`

### P5. kinematics docs 去留 ⬜ 待拍板
- **内容**：(a) 仓库保留副本（开源库惯例）vs (b) 瘦身纯代码库（README 指路上级目录）
- **优先级**：🟢 低

### P6. ARCHITECTURE.md 术语同步 ⬜ 部分完成
- **内容**：① 改名 `SpeedLimiter` → `TwistAccLimiter`
  ② 三级愿景 → v1 范围 + Wheel 承接 S 曲线
  ③ **§2「下行链」表里缺行**：现在只有 Command Interface / SpeedLimiter / Inverse Kinematics / Odometry，
  缺 `contracts` / `wheel` / `chassis_loop`（装配层），且 `Odometry（可选）` 已独立成库
- **优先级**：🟢 低（多数已由 agent 生成 docs 覆盖，待一致性核对）；
  ③ 建议与 `chassis_loop` 落地那一批一起做（见 [`DESIGN.md`](../chassis_loop/docs/DESIGN.md) §11 + 施工单 §5 第 ⑤ 步）

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

### P11. PID 默认语义决策 ✅ **已定案（2026-09-17，随上游 ctlkit 迁移落地）**
- **内容**：评审建议 `limit_out_=0` 默认改为"不限制"；但 wheel 已定案 **"0=disabled"惯例**（max_rate=0 关斜坡、thresh=0 关分离）——两者冲突
- **决策点**：0 语义维持（文档写明坑）vs 改 sentinel vs 分层（0=中立 + preset）
- **影响**：若改，牵动 wheel v0.2.0（已发布 v0.1.0 不可改）
- **2026-09-16 补**：本条现在多了**第二重身份** —— `pid` 仓优化条目 **D3（饱和/限幅状态上报）的硬前置**：
  `limit_out_ = 0` 会把输出**钳死**，于是「输出饱和」标志在**默认配置下恒为 true**（标志会骗人）。
  **P11 不定，D3 就不能落地。** 详见仓外 `~/Develop/Workspace/pid` 的 `optimization_considerations.md` D3。
  顺带：定 P11 的**签名**（加 getter vs 改 `calc()` 返回类型）时，**0 语义自然被一起定下来** —— 反而更容易拍。
- **2026-09-17 决议**：走**分层**方案（第三种）：限幅类 `limit_out_` / `limit_i_` 的 `<= 0` = **不限幅**；
  特性开关类 `thresh_i_sep_ = 0` / `max_rate_out_ = 0` = **关闭**（PID 层）；`Ramp` 自身 `0` = **冻结**不变
  （PID 关斜坡时在构造期归一化为无上限速率，两层不串）。
  随之 **D3 的硬前置解除**：饱和标志改为「与未钳位量比较」，`limit_out_ = 0` 不再让标志恒为真。
  落地位置：上游 ctlkit（vendor 在 `third_party/ctlkit/`，行为契约见其 spec）—— wheel 侧只改配置写法即完成跟随。
- **优先级**：✅ 已结（原 🔴 高）

### P12. 防御校验 ⬜ 未开工
- **内容**：OmniDrive 构造 `wn>6` 越界写 `J[6][3]`、`wn<2` 数学无意义——加断言/参数校验；twist_acc_limiter `reset()` 补测试
- **✅ 已根治（2026-09-17）**：**轮数提成模板参数** `OmniDrive<N>` + `static_assert(N ∈ [3,6])`。
  三次迭代才到位：① 「夹到 [2,6]」→ 被判定**洗白数据**（7 轮的车被静默按 6 轮建模）❌；
  ② 「判无效」（`wn_=0` + `is_valid()` + 零输出）→ 仍靠**调用方自觉**，而「用户会不会遵守」= **不会** ❌；
  ③ **模板参数** → 非法轮数**编译期写不出来** ✅（`wn_` / `is_valid` / 夹紧 / 自检 全部消失）。
  **机器验证**：`kinematics/test/compile_fail/omni_over_capacity.cpp`（故意 `OmniDrive<7>`）+ CMake
  `try_compile` —— 它若编得过（= 守卫被删/放宽），**cmake 配置阶段直接失败**（改坏必红，已实测）。
  调用点 8 处已同步；**KND 仿真输出逐位不变**。
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
- 2026-09-16：**装配层接口定案**（D1~D9，`chassis_loop/docs/DESIGN.md` §8）→ 参考实现与测试落地
  （施工单 §1，12 个变体全变红）；**组件文档隔离**（施工单移入组件 `docs/`，
  长期事实拆出 `DESIGN.md`）；名字维持 `chassis_loop`（**更正了否掉 `ChassisController` 的旧理由**）；
  **关闭 P16**（`reset(Pose)` 早已实现，属过期条目）；**新增 P22**（装配层 `reset()` 语义待设计）；
  修正根 `README.md` 的库清单（拆库后一直没同步：缺 `contracts`/`odometry`、依赖边写的是已删除的那条）；
  门禁 R4（链接存在）**改为对 log 类豁免**（与 R5 同一条理由）
- 2026-09-16（续）：**装配层落地并验收通过** —— `inc/chassis_loop.hpp`（INTERFACE 库，84 行）+ 根聚合器挂载；
  四档编译零告警、聚合 `ctest` **9/9**、13 个变异全部变红、被消费零泄漏；
  `chassis_loop/docs/IMPL.md`（代码地图）+ 三处组件清单（README / AGENTS §5.1 / ARCHITECTURE §2）已同步；
  新增 **P23**（`wheel` 的 `-Wconversion` 隐式转换，验收时撞到）
- 2026-09-17：**通用框架（执行器组接缝）落地** —— 装配层不再认识底盘与轮子（`DESIGN.md` §5.2 / **D11**）；
  `measure` 收设定值、执行器组除轮子外无状态（**D12**）。验收：四档零告警 · 聚合 9/9 ·
  **12 变异全红** · **KND 仿真输出逐位相同**（隔离副本，md5 `9ab64f43…`）。
  上游 `ctlkit` v0.1.1 已 vendor（`SetPwmFn`→`SetEffortFn` 起步）；下游 KND 按决定不动 → **P30**
- 2026-09-17（收尾）：**P25 闭合**（用户手敲 `lim_res_` / `limit_result()` / `t_cmd_final_ = lim_res_.out_`；
  第一版多留一行导致 `limit()` 每拍被调两次 → 加速度上限翻倍，测试红 39 条，已修并复验）；
  **P12 根治**（`OmniDrive<N>` 模板参数 + `static_assert` + **CMake 反例编译测试**，改坏必红）；
  顺手修 `jacobian_apply` 未初始化（`WheelSpeeds out_ws{}`）→ **未用到的槽恒为 0**（F1 同步改写）。
  **lunokhod 进入冻结**：见 `log/HANDOFF.md` 的"冻结快照"。
- 2026-09-17（外部复核）：3 条 —— ① 编译失败（`kMinWheels=3` 与 `2..6` 文案 / `<2>` 测试脱节）**当轮已修**，再根治为「文案不含数字 + **上下界各一个反例编译测试**」（`<2>` 与 `<7>` 都必须编不过）；② `lim_res_` **少写 `{}`** → 真 UB，已修并给 `LimitResult` 补**默认成员初始化器**；③ 补上 P25 缺失的测试（第 10 组：首次 tick 前零值 / 阶跃后饱和标志 / 与 `cmd()` 一致）。
- 2026-09-17（下午）：**外部 agent 的 13 条冻结清单逐条核实** —— 10 条属实、3 条不准
  （P23 的告警数"2 处"实为 1 处；清单 #5"执行器组契约要加口"**不成立**，真身是 P26 的子问题；
  P29 全局名"8 个"实为 15 个）。更正写进 P23/P26/P29；新增 **P31**（下游接入指南）。
  冻结前的收尾顺序见 `log/HANDOFF.md` 末尾。
- 2026-09-17（深夜）：**P32 全仓命名空间政策落地**（用户定 A 案 + 数据放根）。起因：用户发现 P29 只圈了
  `wheel` 一个组件，而 foucault 有 F10 明文"根 + 层子命名空间" → 现状最不一致。10 个库文件包命名空间，
  消费方全部跟上；规则进 `AGENTS.md §3.1`，分层表加「命名空间」列。
  验收：9/9 · 四档零告警 · 全局命名空间已清空（8 个同名全局类型探针）· **KND 逐位不变** · 接入指南真跑。
- 2026-09-17（晚）：**P29 命名空间 + P26 加口/改名 + P23 显式 cast 全部落地**（AI 做，属"加作用域/改名"类）。
  `wheel` 的类型收进 `namespace wheel`（15 个全局名消失）；执行器组契约加第 5 个方法 `effort(i)`（**D13**）；
  `-Wconversion` 严格档全仓零告警。验收：9/9 聚合 · **15/15 变异变红**（含 3 条新口）· 四档零告警 ·
  消费模式零泄漏 · **KND 仿真输出逐位不变**（md5 `9ab64f43…`）。剩 **P25 + P12** 进施工单 `WORK_FREEZE.md`（已归档 `../trash/`：P25 手敲已核、P12 改模板参数根治）。
- 2026-09-16（三）：**运动状态暴露的落地准备** ——
  概念定案「**PWM 是数字、不是硬件；边界是「调用 vs 被调用」**」；
  **PID 算法需求归口仓外 `~/Develop/Workspace/pid`**（登记其 `optimization_considerations.md`
  **D3 饱和/限幅状态上报**，改前已备份到 `/tmp/`）；
  新增 **P25**（装配层接出限幅饱和标志）/ **P26**（轮级 `effort` 暴露 + `SetPwmFn`→`SetEffortFn` 命名中性化）；
  **P3 扩大**为跨仓同源副本 · **P11 增加「pid 仓 D3 硬前置」身份** ·
  **P19 补 §九**（三元组 + 零改动数据入口 + 维度归属 · 明确不新建暴露组件）· **P23 并入 P26**

---

## 2026-09-13 新增（odometry 开工前的尺子审计所得）

### P15. 手写 `odometry/inc/odometry.hpp` ✅ **已完成（2026-09-14）**
- **结果**：`test_odometry` **140/140**，`ctest` **4/4 Passed**，`-Wall -Wextra -Werror` 零告警
- **首次手敲的 3 个坑**（契约字段名不符 / `vy*sin` 写成 `vy*cos` / 同类型字段聚合初始化静默错位）
  **全部被测试抓到** → 规则已入 `kinematics/AGENTS.md` 错误账本
- **收尾**：施工单 `WORK_ODOMETRY.md` 已进 `trash/`；代码地图已追加到 `kinematics/docs/IMPL.md` §9

### P16. `Odometry` 缺初始位姿入口 ✅ **已完成（2026-09-16 核实）**
- **结果**：**早已实现** —— `odometry/inc/odometry.hpp` 有 `reset()` 与 `reset(const Pose&)` 两个重载，
  `odometry/test/test_odometry.cpp` 有对应测试（「初始位姿入口 `reset(Pose)`」）
- **本条为何一直挂着**：2026-09-13 记录时确实没有；2026-09-14 odometry 落地时补上了，但本条没同步划掉
  —— 教训：**代码落地时，要回头看旧待办里有哪些已经被顺手做掉了**
- **原始内容（存档）**：`reset()` 只清零，没有 `reset(const Pose&)` / `set_pose()`；
  设计 §8-#5 “初始 yaw=π/2” 无法直接表达（测试已用“先纯转再走”绕开）
- **优先级**：✅ 关闭

### P17. 两处 2D 位姿积分的积分顺序不一致 ✅ **已统一（2026-09-14）**
- **结论**：`examples/simulation_demo.cpp` 已改为**半隐式欧拉**（先积分 yaw，再用新 yaw 旋转位移），与 odometry R6 同源
- **重要发现**：改动后 **完整输出（含每段终点 + 轨迹图）逐字符相同** ——
  因为 demo 的路径**闭合且对称**，显式的 +φ/2 滞后与半隐式的 −φ/2 超前成对抵消。
  所以统一是**源码同源**问题，不是数值问题
- **顺带更正**：`ODOMETRY_DESIGN.md` §4 原写“半隐式比显式**少一阶误差**”是错的（已用 R14 更正：两者误差**幅值恒等**）

### P18. CI ⬜ → ✅ **已完成（2026-09-14）**
- `.github/workflows/ci.yml`：① 三组件 matrix 编译 + `ctest` ② **金标可复现** job（重跑生成器 → `git diff --exit-code`）
- 三个组件的 `CMakeLists.txt` 都补了 `enable_testing()` + `add_test`（此前 `ctest` 全是空的）

### P19. 一致性残差监测（Consistency Residual Monitoring）⬜ **重新定案（2026-09-15）**

> **2026-09-15 概念重定**：本条原先叫「打滑检测」、归「融合层」。查证后**名字和归属都要改**。
> 一手证据：`reference/robot_localization`（ROS 生态主流融合库）。

#### 一、它不是「打滑检测」，是 FDI 的残差监测

| | 干什么 | 输出 |
|---|---|---|
| **融合 (fuse)** | 多源**合成**更优估计 | 估计量 |
| **残差监测 (monitor / FDI)** | 多源**对比**发现不一致 | 残差 / 权重 |

**打滑检测是后者。**「融合」只在下一步出现（拿残差去**调权重**）。

#### 二、比的是【瞬时量】，不是位移、也不是角度

首选残差：**`wz` 残差**

```
FK(measured_wheel_speeds).wz  −  gyro_z
```

两侧**都是瞬时量，谁都不用推算对方、谁都不用积分**。

- ❌ **不要比 yaw 角**：两侧都变积分量 → 积分是低通，**把要抓的短时残差抹平**；
  陀螺零偏还会积成漂移，**制造假残差**。
- ❌ **不要想「用 IMU 测位移」**：双重积分不可用。**位移比较必须有外部绝对参照**
  （视觉 / 激光 / GNSS），那是另一个层次，也是行星车实际用的办法。

**注意**：`twist.wz` 是 `Odometry::update` 的**输入**，不是输出 —— 所以这个比较
**不需要 odometry 参与**，也不需要融合器。**两个来源自己就能比。**

#### 三、成熟界怎么做（本条最重要的调研结论）

`reference/robot_localization` 里：

- `filter_base.cpp` `checkMahalanobisThreshold()` —— 创新（innovation = 量测 − 预测）的**马氏距离**门限
- `ekf.cpp` 调用点 `if (通过) { state_ += K·innovation }` —— **没有 else，失败即整条量测丢弃**
- `ros_filter.cpp`：**每个 topic 一个门限参数**
  （`odom0_pose_rejection_threshold` / `odom0_twist_rejection_threshold` / `imu0_accel_rejection_threshold` …）
- `measurement.hpp`：默认值 `std::numeric_limits<double>::max()` → **默认永不拒绝**

**→ 成熟系统里没有「打滑检测组件」，它是融合器的一个【可选门限参数】。**

另外找到的三个专门库（`UMich-CURLY/slip_detection_DOB` · LiDAR+IMU autonomous racing ·
祝融号火星车 GPR 打滑模型）**全是绑死特定平台/传感器的研究代码，没有通用可移植的**。

#### 四、输出应该是【权重】，不是 `is_slipping` 旗标

旗标只能**报警**；权重才能让下游**用它**。而「不可信怎么办」永远是**使用者**的决定。

#### 五、归属：库里只到「交事实」为止

| 层 | 职责 | 归属 |
|---|---|---|
| **① 可观测量出口** | 暴露 `cmd()` / `twist()` / `wheel_speed(i)` / `wheel_target(i)` / `pose()` / `yaw_ref()` | ✅ **lunokhod 装配层**（P20）；`odometry` 已有的 `SampleSink` 是同一件事的另一条路 |
| **② 残差生成器** | `twist().wz_ − gyro_z`（需要库外的标量） | ❌ **库外**（上层 / PC 工具） |
| **③ 门限 → 权重** | 阈值是参数，要标注数据标定 | ❌ 库外 |
| **④ 用权重** | 融合器 / 消费方 | ❌ 需要融合器，**现在不存在** |

**库内的活儿到 ① 就结束了。**

#### 六、明确不做

- ❌ **不做通用「打滑检测组件」** —— 成熟做法是门限参数，不是模块
  （判据：①第二个消费者？无 ②变更节奏独立？无 ③独立可测内容？有，但那是判定策略，属上层）
- ❌ **不输出 `is_slipping`** —— 输出权重
- ❌ **不比位姿 / 角度** —— 积分是低通
- ❌ **不在库里做判定** —— 红线：**库只交事实，不做决定**（见 `../AGENTS.md`）

#### 七、L0 改名：它不是打滑检测

| 原 L0「cmd vs twist」 | **实际检测的是执行器故障 / 轮间一致性** |
|---|---|
| 轮子**打滑**时 | 编码器**跟着转** → `twist ≈ cmd` → **看不出整车打滑** |
| 轮子**堵转/卡死**时 | `twist ≈ 0` 而 `cmd ≠ 0` → 报警 ✓ |
| **单轮**打滑 / 缺气 | **轮间不一致**看得见 ✓（只要编码器就够） |
| **整车**打滑（全部轮子同时滑） | ❌ **编码器上不可见** —— **必须有外部参照**（陀螺 / 加速度计 / 视觉） |

对应落点：**单轮的「自己 vs 自己的指令」可以进 `wheel`**
（`speed_cmd_` / `speed_cur_` 都在它内部），但**它只看得见自己** ——
**轮间一致性做不了**（`Wheel` 是每轮一个实例）。
而且进 `wheel` 的话，**暴露残差数据，不做判定**（与 odometry 的红线一致）。

#### 八、前置条件（仍未被解开）

这是**异常检测**，阈值不能手拍 —— 需要「正常」与「打滑」的**标注数据**，现在两份都没有。

**建议路径**：先在 PC 侧做 —— 用 **P20 装配层的可观测出口**导 CSV + `foucault` replay 导 IMU，
时间戳对齐 → Python 里试算法 / 定阈值 → 验证过再考虑落到嵌入式。

- **优先级**：🟡 中（**被「先采数据」卡住**；而采数据的入口正是 P20）

#### 九、2026-09-16 补充：落地清单 + 数据入口确认

**1. 能分类「打滑 / 堵转」的最小数据集 = 三元组**（缺一个就分不开）：

| 量 | 来源 | 现状 |
|---|---|---|
| `wheel_target(i)` 目标 | 逆解输出 | ✅ 已有 |
| `wheel_speed(i)` 实测 | 编码器（`MeasureSpeedFn`） | ✅ 已有 |
| `wheel_effort(i)` 努力度 | PID 输出（现在**没有出口**） | ❌ → **P26** |

理由：**堵转与空转打滑在转速上是同一个观测**（都表现为「跟不上指令」），
**只有加上「努力度」才分得开**（堵转 = 力到顶 + 转速≈0；空转 = 力不大 + 转速正常）。

**2. 「先采数据」这一步【零改动】可做**（这是 §八 那个卡点的解）：
`SampleSink` 是在 `odom_.update()` 内部**同步调用**的，而 `odom_.update()` 是 `tick()` 的**最后一步**
→ **在 sink 回调里读 `chassis_loop` 的只读口，拿到的就是同一拍的值**（不需要额外缓冲、不需要时间戳对齐）。

**3. 明确不做（本轮再次确认，别重开）**：**不新建「运动状态暴露」组件** ——
数据全在，多一层转发只会引入同步 / 生命周期问题；形态就是「**sink + 只读口**」。
判据同 `chassis_loop/docs/DESIGN.md` §2 拒绝预留 `δcmd`（**为不存在的消费者开 API = 猜**）。

**4. 维度补全（本轮盘过、确认属库外的）**：

| 维度 | 归属 | 备注 |
|---|---|---|
| **倾角 pitch/roll** | foucault + 应用层 | **30° 坡必需**（水平位置多报 15.5%，见本文档「坡带来的两个后果」）；`chassis_loop` 不碰姿态 |
| **轮间一致性** | 库外 | `Wheel` 每轮一个实例，看不见「轮间」；装配层只给事实，判定在外 |
| **位移残差** | 库外 | 需外部绝对参照（视觉 / 激光 / GNSS） |

**5. 残差的第一消费者其实是「标定」，不是「打滑」**：
标定误差 = **长期一致的系统性残差**（轮径 / 轮距 / `twist_scale_` / IMU 安装对齐）；
打滑 = **瞬时残差**。两者**共用同一套采样**，但**判定方法完全不同**
（长时最小二乘 / 已知基线 vs 瞬时门限）。**标定门槛最低、立刻能用**（G6 缺口）。

---

## 2026-09-15 新增

### P20. 装配层（Chassis Loop）🔴 **下一任务**（2026-09-15 定）

- **问题**：`lunokhod` 的零件齐了，但**装配逻辑住在消费方** ——
  `~/Develop/Workspace/fw_poc/`（`main.cpp` 110 行；已退役）与 `~/Develop/Workspace/KND_Trial/`（当时的
  `firmware/`，此后重整为 `app/` + `bsp/` + `sim/`）
  **写的是同一条链**。链里有易错知识（tick 顺序、`dt` 用实测值、融合钩子在里程计之后），
  每重写一遍就有一次写错的机会。
- **它的价值不只是「能把车跑起来」** —— 它同时是整条**感知线的取样点**（P19 的全部原料）
- **范围（钉死）**：**只做下行链 + 里程计**
  ```
  限幅 → 逆解 → 4×Wheel → 正解 → 里程计
  ```
  **不碰姿态融合** —— 碰了 lunokhod 就要 `#include "estimator.hpp"`，两个库互相认识 ✗
- **判据**：**装配层只吃「已算好的量」（`Twist` / `dt`），不碰 IO。**
  一旦它需要知道「命令从哪来」「IMU 在哪读」 → 那不是装配层，是应用层。
- **必须开出的只读口**（P19 的数据入口，顺便验证范围没越界）：
  `pose()` · `yaw_ref()` · `twist()`（FK 输出）· `cmd()` · `wheel_speed(i)` · `wheel_target(i)`
- **验收（可机器判）**：把 `~/Develop/Workspace/KND_Trial/firmware/` 里的 `app.cpp` 换成用装配层，
  **仿真输出逐位不变**（`pose=(0.3425, 0.1085) yaw_odo=0.2574 …`）
- **已定案**（2026-09-15 名字/位置；2026-09-16 接口 D1~D8 全定案）：
  名字 `chassis_loop` / 位置仓库根级 / 不预留 `set_correction`，接口层 D1~D9 全定案
  —— 见 [`DESIGN.md`](../chassis_loop/docs/DESIGN.md) §8
- **验收结论（2026-09-16）**：✅ **已通过**（含验收锚点第 ⑥ 步 —— KND_Trial 仿真输出 **md5 逐位相同**）。
  完整记录见 [`chassis_loop/docs/log/ACCEPTANCE.md`](../chassis_loop/docs/log/ACCEPTANCE.md)；
  代码地图见 [`chassis_loop/docs/IMPL.md`](../chassis_loop/docs/IMPL.md)。
- **遗留**：① **P24**（轮子容量 < 契约容量 → `OmniDrive` wn=6 越界，真 UB，待拍板修法）；
  ② 施工单已进 `trash/`（契约与决策已入 `chassis_loop/docs/DESIGN.md`）；③ P22（`reset()` 语义）
- **遗留**：`reset()` 语义未定 → 见 P22
- **优先级**：🔴 **高（下一任务）**

### P21. Swerve Drive 可行性复核 ⬜ **记入待办**（2026-09-15 提出）

- **触发**：核对 `kinematics/docs/DESIGN.md`「刻意不覆盖的范围」时发现
  **Swerve 的排除理由是错的**。
- **查证**：`kinematics/docs/THEORY.md` §2.4 自己给的公式
  ```
  φ_i = atan2(Vy + ω·x_i,  Vx − ω·y_i)
  v_i = sqrt((Vx − ω·y_i)² + (Vy + ω·x_i)²)
  ```
  **这是标准的一对一瞬时映射，零隐藏状态。** `Twist` 进，`(φ, v)` 出。
- **DESIGN.md 写的「有跨帧状态（180° 翻转管理）」是什么**：`(φ, v)` 与 `(φ+π, −v)`
  产生同样的运动，选哪个只是为了**让舵机少转** —— 那是**轮控层的优化，不是运动学**。
- **真正的障碍**：`WheelSpeeds` 只有一个 `float values_[6]` —— Swerve 每模块要出
  **两个量**（角度 + 速度），**契约装不下**。
- **所以分类应改成**：

  | | 输入契约 | 输出契约 | 验证标准 |
  |---|---|---|---|
  | **Swerve** | ✅ 不用改（3 个自由度全用得上） | ⚠ **需要装 `(φ_i, v_i)`** | ✅ 不用换（输入空间完整、互逆性成立） |
  | **Ackermann** | ⚠ 需要新输入类型 | ⚠ 需要装 `(δ, v)` | ⚠ **必须换**（可达子空间缺「原地转」+ `vx→0` 奇点） |

  → **Swerve 是「契约太窄」，Ackermann 是「抽象不成立」。**
- **优先级**：🟢 低（先把装配层和固件做完）

### P22. 装配层的 `reset()` 语义 ⬜ **待设计**（2026-09-16 提出）

- **来源**：装配层 [`DESIGN.md`](../chassis_loop/docs/DESIGN.md) §8.2 的 D7 定案「v1 不做」，但留下了一个洞。
- **洞在哪**：各积木都有清零入口（`TwistAccLimiter::reset()` · `PID::reset()` ·
  `SmoothPlanner::reset()` · `Wheel::stop()` · `Odometry::reset(Pose)`），
  但它们都是 `ChassisLoop` 的 **private** 成员 → 调用方**一个都够不着**（连 `Wheel::stop()` 也够不着）。
- **为什么没现在做**：急停/复位路径还没设计，光「要不要顺带 `stop()` 那 4 个 `Wheel`」就是两难 ——
  写 PWM 0 会碰**未用到的轮子**（与决策 3 冲突）；只停前 `n` 个又不行，
  因为 `reset()` 很可能在第一次 `tick()` **之前**被调，那时 `target_ws_.count_` 还是 0。
- **注意**：它**不是**热切换配置（换参数需要 `set_config()`，那个不做 —— 同 `DESIGN.md` §8.1 的判据）。
- **下一步**：等固件的急停 / 失效保护（缺口 G7）立项时一并定；先定「谁在什么条件下调它」。
- **优先级**：🟡 中（不阻塞装配层落地）

### P23. `wheel` 在 `-Wconversion` 下有一条隐式转换 ✅ **已修（2026-09-17）**

- **来源**：验收 `chassis_loop` 时跑「严格档」撞到的 —— **不是那一批代码的问题**。
- **现象**：`actuator/wheel/src/wheel.cpp` 的 `set_pwm_(motor_id_, out_pwm)`：`float` → `int16_t` 隐式转换，
  开 `-Wconversion` 时 `-Werror=float-conversion` 直接挂。
- **性质**：**功能上是有意的** —— PWM 接口就是整数，且 `out_pwm` 已被 `PIDConfig::limit_out_` 夹住（默认 1000），
  不会溢出。问题只在「有意」看不出来（隐式截断）。
- **选项**：① 显式 `static_cast<int16_t>`（一行，表达意图）；② 加饱和钳位再 cast；③ 不管
- **影响面**：现状**不影响 CI**（各组件自测只用 `-Wall -Wextra -Werror`），但谁想开 `-Wconversion` 编整仓就会卡住。
- **2026-09-16 补**：**并入 P26 一起修** —— 落 `effort_` 时那两处调用本来就要改
  （`set_pwm_` → `set_effort_` + 显式 `static_cast<int16_t>`），顺手关闭本条。
- **2026-09-17 核实（外部清单说"2 处"，要纠正）**：调用点**确实是 2 处**（`Wheel::update` 里
  的正常下发 + 停机那处的 `0.0f`），但**只有 1 处真告警** —— 常量 `0.0f` 不触发
  `-Wfloat-conversion`（实测整仓严格档只报这一条）。修的时候两处都显式 `cast`，但别指望它变 2 条告警。
- **优先级**：🟢 低（一行的事，属「跨组件一致性」）

### P24. 装配层的轮子容量 < 契约容量 → 越界 ✅ **已修（2026-09-16）**

- **内容**：契约 `WheelSpeeds.values_[6]` 允许 **6** 轮，`ChassisLoop` 却只持有 `Wheel* wheels_[4]`。
  `tick()` 按 `count_` 循环 `wheels_[i]` → `ChassisLoop<OmniDrive>`（wn = 6）**越界写**。
- **证据（实测）**：UBSan `index 4 out of bounds for type 'Wheel *[4]'` + ASan `SEGV`（在 `Wheel::set_cmd`）→ 段错误。
- **为什么没被测出来**：7 组测试只用 `MecanumDrive`(4) 与 `DiffDrive`(2)，`OmniDrive` **一次都没构造**。
- **影响面**：当时消费方（KND_Trial 麦轮 / `fw_poc`）都是 4 轮 → 触发不到；但 `OmniDrive` 本来就支持 N 轮、就在本仓。
- **选项**：① **容量对齐契约**（`wheels_[6]`；2 轮车多几个死对象，改动最小）
  ② 加守卫（编译期/运行期断言，明确「本组件只支持 ≤ 4」）
  ③ 容量做模板参数 `ChassisLoop<Chassis, N>`（零浪费，但要 `index_sequence` 构造 N 个 `Wheel`）
  ④ 真 N 泛化（给 `Wheel` 加默认构造 / `std::array` + 工厂）
- **附带**：测试要补「边界 N」用例（N = 契约上限、N = 1）—— 否则「支持 N」只是口头声明。
- **修法（2026-09-16 定案）**：**容量对齐契约 = 6**（`wheels_[6]`）—— 见 `chassis_loop/docs/DESIGN.md` §8.2 **D10**；
  配套补了**边界 N** 测试（三轮 / 六轮），并验证「容量退回 4 → 新用例在 ASan 下必然变红」。
  **未选**：容量做模板参数（零浪费，要 `index_sequence`）· 真 N 泛化（要给 `Wheel` 加默认构造）—— 留到真计较 RAM 时。
- **优先级**：✅ 关闭

---

## 2026-09-16 新增（PID 归口 + 运动状态暴露）

### P33. 接缝写错时的**诊断质量**（解冻后候选，不阻塞冻结）⬜ **未做**

- **来源**：2026-09-17 拿真下游（`KND_Trial` 隔离副本）对着冻结版实测撞出来的 ——
  旧代码 `ChassisLoop<MecanumDrive>`（模板参数误写成底盘）产出 4 条错误，**全部指向 lunokhod 自己的
  `chassis_loop.hpp`**：「`MecanumDrive` has no member named `inverse` / `apply` / `measure` / `forward`」。
  用户看到的是"我写的类缺方法"，而真因是"模板参数该是执行器组"。
- **代价**：每个从旧版升上来的消费方都要自己猜一次（本轮已加进 `docs/INTEGRATION.md` §6 的坑列表兜住）
- **建议做法**：在 `ChassisLoop` 类体**最前面**放一条 `static_assert`（文案直接说"模板参数应为执行器组，
  见 `wheel_set.hpp` 的 `WheelLoop<Chassis>` 别名"，并列出 5 个方法名）——
  报错顺序在最前面，用户第一眼就能看到。**不引入 `<type_traits>`**（库仍是 `<cmath>`/`<cstdint>` 零依赖）。
- **判据**：故意把模板参数写成 `MecanumDrive`，第一条错误必须是那条 `static_assert`。

### P25. 装配层暴露限幅饱和标志（`LimitResult` 被丢弃）✅ **已做（2026-09-17）**

- **内容**：`chassis_loop::tick()` 里 `t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_;` —— **只取输出**，
  `LimitResult` 的 `is_vx_lim_` / `is_vy_lim_` / `is_wz_lim_` 三个标志**算完就丢**。
  它们是「上层要的速度超出加速度能力」的第一手证据，**零算法成本**（信息早就算出来了）。
- **改动位置**（`chassis_loop/inc/chassis_loop.hpp` —— INTERFACE 库，**只此一个文件、3 处**）：
  1. `tick()`：`lim_res_ = limiter_.limit(t_cmd_in_, dt); t_cmd_final_ = lim_res_.out_;`
  2. getters 区：`LimitResult limit_result() const { return lim_res_; }`
  3. private 区：`LimitResult lim_res_{};`
     ⚠ **必须带 `{}`** —— `LimitResult` / `Twist` 都是**无默认成员初始化器的聚合类型**：
     写 `LimitResult lim_res_;` 是**默认初始化 = 成员为垃圾值**（同 `WHEEL_LESSONS` #7 的坑）。
- **可选去冗余**：`t_cmd_final_` 与 `lim_res_.out_` 是**同一份数据的两个副本**；
  想干净就删 `t_cmd_final_` 改用它（`cmd()` / `inverse_kinematics` / `odom_.update` 共 5 处小改）。
- **顺手**：`tick()` 里那行残留注释 `// --- process to res残差 ---`（它下面实际是 `odom_.update`）——
  删掉，或改成「此处不留残差，残差在库外」。
- **不用改** `twist_acc_limiter` —— 它**已经交了** `LimitResult`，问题只在装配层没接。
- **与接缝的关系（2026-09-17）**：接缝落地后，`tick()` 里那行 `limiter_.limit(...)` 改叫
  `t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_;` —— **位置没变**，照原计划 3 处即可。
- **测试**：AI 写（判别力：故意不接 `lim_res_` → 标志恒 false → 必须变红）。
- **优先级**：🟡 中（3 行改动、零风险；是 P19 采数据的字段之一）

### P26. 轮级「努力度」（effort）暴露 + `SetPwmFn` 命名中性化 ✅ **已做（2026-09-17）**

- **背景**：分类打滑/堵转需要**三元组** `目标 / 实测 / 努力度`。前两个装配层已有
  （`wheel_target(i)` / `wheel_speed(i)`），**第三个没有出口** —— `Wheel::update()` 里 `out_pwm`
  交给 `set_pwm_` 就没了。而它是**唯一能分开「堵转」与「空转打滑」的量**（只有速度分不开：两者都是「跟不上指令」）。
- **概念澄清（本轮定案）**：**`SetPwmFn` 不是「碰了硬件」，而是「命名绑定了实现」**。
  `SetPwmFn` 是**注入的回调**，库连「这是 PWM」都不知道（对面可能是 TIM 寄存器 / PC 模型 / 日志文件）。
  **硬件的边界是「调用 vs 被调用」，不是「数字 vs 硬件」**。
  **真·PWM 实现（`hal::set_pwm`）叫 `set_pwm` 完全正确** —— 错的只是把它写进**库的接口**：
  库的接口属于**所有**调用方。这条与 `WHEEL_LESSONS` §5 的规矩一致
  （「命名中性（`MeasureSpeedFn` / `target_speed_`），单位由调用方约定」）——
  `MeasureSpeedFn` 做到了中性，`SetPwmFn` 没有。
- **改名方案**：`SetPwmFn` → **`SetEffortFn`**（control effort，控制论术语、单位无关）。对称性论据最强：

  ```
  MeasureSpeedFn  →  speed_cur_  →  get_speed()
  SetEffortFn     →  effort_     →  effort()
  ```

  动词 + 物理量，**一进一出完全对称**。
- **改动位置（3 个活代码文件）**：
  - `actuator/wheel/inc/wheel.hpp`：`+ float effort() const;` · `+ float effort_;` · `SetPwmFn`→`SetEffortFn` · `set_pwm`→`set_effort`
  - `actuator/wheel/src/wheel.cpp`：初始化列表 `+ effort_(0.0f)` · `update()` 里存 `effort_` ·
    `stop()` 里清 `effort_` · 两处 `static_cast<int16_t>`（见 P23）
  - `chassis_loop/inc/chassis_loop.hpp`：`+ float wheel_effort(uint8_t i) const` · `using SetPwmFn`→`using SetEffortFn`
- **四个坑**：① `effort_` **必须进初始化列表**（`WHEEL_LESSONS` #7）
  ② `stop()` 里也要清（否则 stop 后还留着上一拍的值）
  ③ 注释写明语义 = 「**本拍 PID 输出**」，不是「实际写到硬件的值」（回调为空时仍有值）
  ④ 量纲由调用方约定 → 文档声明 + **测试钉单位**（`WHEEL_LESSONS` §5）；
  **不做归一化**（那要库知道 `limit` 的语义 → 撞 P11）
- **对调用方零影响**：`KND_Trial` / `fw_poc` 的 `hal::set_pwm` 是**位置传参**，不用动（`fw_poc` 此后已并入 `KND_Trial`）。
- **A 类日志会留旧名**（`odometry/docs/log/ODOMETRY_FAQ.md`、`actuator/wheel/docs/log/WHEEL_LESSONS.md`）——
  **正常，不许回头改**（日志只增不改）。
- **时机**：**不单独发版** —— 与 **P11**（0 语义）+ **P23**（-Wconversion）+ effort 一起进 **wheel v0.2.0**，
  一次版本跳变解决四件事。**现在只有 3 个引用文件，是改名最便宜的时候**（引用点只会越来越多）。
- **测试**：AI 写（判别力：故意不存 `effort_` → getter 恒 0 → 必须变红）。
- **进度（2026-09-17）**：**类型别名已改**（`SetPwmFn` → `SetEffortFn`，`wheel.hpp`/`wheel.cpp` + 接缝的
  `wheel_set.hpp` 都用了新名）；**剩下**：`float effort() const` + `effort_` 成员 + 调用点局部变量名
  `set_pwm` → `set_effort` + 装配层 `wheel_effort(i)`。已实测：改名**没有破坏任何目标**
  （`test_wheel` / `example_wheel` / 其余组件全过；下游 `hal::set_pwm` 是位置传参、不用动）。
- **⚠ 子问题（2026-09-17 立）**：装配层要不要出 `wheel_effort(i)`？
  **现状够用**：`wheel_speed(i)` / `wheel_target(i)` 读的是装配层**自己的**两份 `WheelSpeeds`
  缓存，**不需要**执行器组的 `wheels_`（它 private 也不影响）—— 所以**不是**"契约缺个口"。
  真问题是：`effort` 这种东西**不在** `WheelSpeeds` 里，装配层**永远拿不到**。
- **✅ 决定（2026-09-17，用户拍板）**：**加口** —— 执行器组契约第 5 个方法 `int16_t effort(uint8_t i) const`
  （决策 **D13**），装配层出 `wheel_effort(i)`。理由：下游构建期就要单轮状态暴露。
  实现落点：`wheel::Wheel::effort()` + `WheelSet::effort(i)` + `ChassisLoop::wheel_effort(i)`。
- **✅ 同批做完**（改名 + effort + P23 的显式 cast，隔离副本上 KND 输出**逐位不变**）：
  `SetEffortFn set_effort` / `set_effort_` / `out_effort` / `effort_` / `effort()`；
  `float→int16_t` 全部显式 `static_cast`（`-Wconversion` 零告警）；示例的 `set_pwm` 回调 → `set_effort`。
- **优先级**：🟡 中（P19 的前置之一；改名本身不紧急，但拖延变贵）

---

## 2026-09-16 新增（分层整理 + 通用框架立项）

> ⚠ 编号避开 P25（限幅饱和标志）/ P26（effort + `SetPwmFn` 改名）—— 那两条是**同一个文件**
> （`chassis_loop/inc/chassis_loop.hpp`）的待手敲项，与接缝改造**互补不冲突**：
> 接缝版只是把 `limiter_.limit(...)` 的调用点挪了个位置，P25 要接的 `LimitResult` 照样接得上。

### P27. FOC 作为执行器：接缝细化 ⬜ **待立项**（2026-09-16）

- **两种形态，接缝吃法不同**：
  - **电流环跑在驱动里（推荐）**：FOC = HAL 之下的实现 —— `set_pwm(id, ±1000)` 的 effort
    语义按「目标转矩」解释即可 → **装配层与 `Wheel` 都不用改**。
  - **FOC 自带速度环**（对外只收「速度模式」）：它替换的是「轮控」那一格 →
    写一个 `FocSet` 实现执行器组契约（[`../chassis_loop/docs/DESIGN.md`](../chassis_loop/docs/DESIGN.md) §5.2），
    **装配层一个字不改**。
- **接缝已落地（2026-09-17）**：`ActuatorSet` 契约见 `chassis_loop/docs/DESIGN.md` §5.2（决策 **D11**）——
  届时照它写一个 `FocSet` 插进 `ChassisLoop<FocSet>` 即可，装配层不动。
- **要定的第一件事**：**速度环放在哪一层**（`Wheel` 里 / FOC 驱动里）—— 它决定要不要 `FocSet`。
- **与 P7（FOC 重构）相邻**；素材：`cyclotron`（FOC 项目，已冻结）。
- **优先级**：🟢 低（等真做 FOC 接入）

### P28. Swerve 作为执行器组：**输出契约要一起扩** ⬜ **待立项**（2026-09-16）

- **接缝本身已装得下**（写一个 `ModuleSet` 实现那四个方法即可）—— 但**那不够**：
- **真正的障碍是契约**（同 **P21**）：`WheelSpeeds` 只有 `float values_[6]`，装不下每模块的
  `(角度, 速度)`；而且 `odometry` 的记录契约 `OdometrySample.ws_` **也是** `WheelSpeeds`。
- **要一起动的四处（同一次改动，先改设计）**：① `contracts` 新增模块级契约
  ② `kinematics` 的 Swerve 实现 ③ `odometry` 的记录字段 ④ 执行器组的
  `Setpoints` / `Feedbacks` 类型别名（`DESIGN.md` §5.2 现在**有意**把它们钉在 `WheelSpeeds`）。
- **接缝的形状不用改** —— 这正是 **D11** 抽对了的证据（接缝已于 2026-09-17 落地）。
- **优先级**：🟢 低（等 Swerve 立项，见 P21）

## 2026-09-17 新增（上游算法库接入）

### P29. `wheel` 组件加命名空间（接口卫生）✅ **已做（2026-09-17）→ 已被 P32 推广到全仓**
- **内容**：`actuator/wheel/inc/{pid,lpf,ramp,smooth_planner}.hpp` 里的类型**全在全局命名空间**
  （`PID` / `PIDConfig` / `LPF` / `Ramp` / `SmoothPlanner`）—— 谁 include，谁被注入全局名。
  实测**共 15 个**（外部清单写"8 个"，只数了 `pid.hpp` 一家）：
  **转发名 11** = `pid.hpp` 8 个（`PID` / `PIDConfig` / `PIDGains` / `PIDLimits` / `PIDTunings` /
  `PIDState` / `PIDStatus` / `PIDPorts`）+ `lpf.hpp` `LPF` + `ramp.hpp` `Ramp` +
  `smooth_planner.hpp` `SmoothPlanner`；**`wheel` 自有名 4** = `Wheel` / `SmoothPlannerConfig` /
  `MeasureSpeedFn` / `SetEffortFn`。
- **危害**（不是「代码坏了」，是接口债）：
  1. 名字太通用 → 消费方自己定义一个 `PID` 就撞名；「一边 `using` 一边在别处自己定义」会落到 **ODR 隐患**（不报错的那种）
  2. 头文件污染全局命名空间
  3. 多库协同时无法消歧（`wheel::PID` vs `ctl::PID`）
- **来由**：血缘是单工程嵌入式 C++（无 namespace 习惯）；cyclotron 侧搬运时加了 `foc::algo`，本仓一直没加。
- **迁移后的现状（2026-09-17）**：风险**未变差**（迁移前这些名字本来就在全局）；但**修起来变便宜了** ——
  定义已不在本仓（上游 ctlkit vendor 在 `third_party/ctlkit/`），收口只需改 4 个转发头
  （全局 `using` → `namespace wheel { using ctl::PID; … }`），或干脆让消费方直接写 `ctl::PID` 并删掉转发头；
  消费方漏改会被编译器逐条点名（不静默）。
- **影响面**：`wheel.hpp` / `chassis_loop` / `fw_poc`（后并入 `KND_Trial`）/ `KND_Trial` 的限定名；破坏性变更 → 建议与 P26 同批进 wheel 的下一次发布
- **✅ 已做（2026-09-17，趁下游未开工）**：15 个全局名全部收进 `namespace wheel`；
  4 个转发头里的**过期注释**（原文写着"保留全局名"）同批改掉；调用点：`wheel_set.hpp`（限定名）、
  两个测试与示例（文件内 `using`）、CI 消费示例、`docs/INTEGRATION.md` 示例。
  反向验证：写一个"全局再定义 `PID`/`Wheel`/`LPF`/`Ramp`"的探针，不再撞名。
- **⚠ 载体问题**：`wheel` 组件**没有 B 类设计文档**（只有教训日志 `docs/log/WHEEL_LESSONS.md`），
  所以"类型在 `wheel::` 里"这条约定目前记在 `chassis_loop/docs/DESIGN.md` §5.2 与 `docs/INTEGRATION.md` §2。
  将来若给 `wheel` 补 `docs/DESIGN.md`，把这条连同 `LPF`/`Ramp`/`SmoothPlanner` 的对外约定一起搬过去。
- **优先级**：🟢 低（不急，但趁转发头还在时改动面最小）

### P31. 下游接入指南（一份完整调用）🟡 **待做**（2026-09-17 提出）

- **缺什么**：五个库各自有测试与 `examples/`，但**装配层没有端到端示例，也没有"平台层要实现什么"的说明**。
  要接下游（固件 / 别的项目）的人拿到 `chassis_loop` 之后，**没有一份可照抄的完整装配**：
  ① `MeasureSpeedFn` / `SetEffortFn` 的语义与单位（谁负责符号与量程）② 怎么选底盘、怎么组 `WheelSet`
  ③ 主循环怎么写（`dt` 从哪来、`now` 从哪来）④ 数据怎么导出（`SampleSink`）⑤ 姿态怎么接（foucault 只管标量）
  ⑥ 常见坑（未用到的轮子不许被碰、`count_` 的边界、单位/符号约定）。
- **为什么不做成组件内的 `examples/`**：它不是"这个库怎么用"，是**"整条链怎么装起来"** ——
  跨 5 个库 + 姿态库，属于**仓级**文档（放 `docs/`），不是组件文档。
- **判据（写没写够）**：一个没读过本仓的人，照着它能在 STM32 上把 `tick()` 跑起来。
- **状态（2026-09-17）**：已起草 `../docs/INTEGRATION.md`（示例代码**真编译并跑过**）。
- **优先级**：🟡 中（转战下游前完成）

---

### P32. 全仓命名空间政策（A 案）✅ **已落地（2026-09-17）**

- **来源**：用户发现 P29 的范围圈得太窄 —— foucault 有**明文决策 F10**（根 `foucault::` + 按层子命名空间），
  而 lunokhod 只给 `wheel` 一个组件加了命名空间、其余全在全局 → **一半全局一半 `wheel::`，是最不一致的状态**。
- **定案（用户拍板 A + 数据放根）**：

  | 名字 | 命名空间 |
  |---|---|
  | `Twist` / `WheelSpeeds` / `Pose`（跨库公共词汇） | **`lunokhod::` 根** |
  | 底盘几何 | `lunokhod::kinematics` |
  | 里程计与记录 | `lunokhod::odometry` |
  | 限幅 | `lunokhod::twist_acc_limiter` |
  | 轮控 | `lunokhod::wheel` |
  | 编排 / 执行器组 | `lunokhod::chassis_loop` |

- **理由**：与 foucault F10 一致；**根名防撞名**（消费方自己的 `kinematics::` 不会撞）；上层一眼看出类型归属。
- **落地内容**：10 个库文件包命名空间（**内部引用不逐处加前缀**，靠外层查找）· 跨组件引用写兄弟子命名空间
  （`odometry::SampleSink` / `twist_acc_limiter::TwistAccLimiter`）· 消费方（测试/示例/工具/CI 示例/接入指南）用 using。
- **规则写进** [`../AGENTS.md`](../AGENTS.md) **§3.1**（唯一来源）+ [`ARCHITECTURE.md`](ARCHITECTURE.md) §0.1 分层表加了「命名空间」列。
- **验收**：聚合 9/9 · 四档零告警 · 全局命名空间**已清空**（探针：全局再定义 8 个同名类型可共存）·
  **KND 仿真输出逐位不变**（md5 `9ab64f43…`）· 接入指南示例走 CMake 链真编真跑。
- **优先级**：✅ 已闭合（趁下游未开工，与 P29 同一个窗口）

---

### P30. 上游破坏性改动后，外仓消费方没有自动化核对 ✅ **已做（2026-09-17 手工跑通）**

- **怎么发现的**：`ctlkit` v0.1.1 把 `PIDConfig` 改成 `gains_/limits_/tunings_` 三段后，
  **KND_Trial 的 `app.hpp` 直接编译不过**（还在用旧扁平字段 `p.kd_ = …`）。
  lunokhod 自己的 5 个 job 全绿 —— 因为 **KND_Trial 在库外，且尚未提交任何 commit**。
- **当前状态（2026-09-17）**：**未修** —— 用户决定「只改 lunokhod，先不管下游」。
  修法很小：那 10 行 `[](){…}()` 换成一处具名链式调用（`PIDConfig{}.kp(8.0f).ki(1.2f)…`，数值不动）。
- **风险同类**：任何「下游只改调用点」的上游迁移（ctlkit 升版、契约改名、`SetPwmFn`→`SetEffortFn`）
  都会重演这一幕：**库内绿、库外断**。
- **2026-09-17 补：下游有两个**（都在库外、都未提交）—— `KND_Trial`（3 个文件）与
  ~~`~/Develop/Workspace/fw_poc/`（1 个文件）~~ —— **2026-09-17 核实：它的内容已全部并入 `KND_Trial`**
  （两份手写链的仿真输出 md5 逐位一致），**已不再是消费方**，不必迁移。
  **全仓命名空间（P32）又是一次同类破坏**：
  两个下游都要加 `lunokhod::` 限定。**核对清单第 2 条要跑两遍**。
- **选项**：① 每次上游迁移后**人工跑一次** KND sim 构建 + 锚点比对（约 1 分钟，最省）
  ② 给 KND_Trial 建仓并加一个「消费 lunokhod main」的 CI job（跨仓，成本高）
  ③ 在 `third_party/ctlkit/VERSION` 的「校验」一行旁补一条「下游手动核对清单」
- **建议**：先 ① + ③（把动作写进文档，不建跨仓 CI）。
- **2026-09-17 部分落地**：③ 已做 —— 核对清单写进了 `third_party/ctlkit/VERSION` 的「下游核对」
- **✅ 2026-09-17 收口（按选项 ① 真跑了一遍）**：在 `/tmp` 隔离副本上把真下游 `KND_Trial`
  迁到冻结版，随后**用户授权直接改真工程**（只动 lunokhod 相关的 3 个文件：
  `app.hpp` / `app.cpp` / `main.cpp`；约 60 行）。
  验收：`knd_sim` 输出与 `chassis_loop/test/golden/knd_sim_anchor.txt` **逐位一致** ·
  KND host 测试 **3/3** · 板级固件交叉编译**过**（`arm-none-eabi`，2.3 MB `.elf`）· 零告警。
  **摩擦与改进已回灌文档**：接入指南 §6 新增第 6 条坑（`ChassisLoop` 模板参数应为执行器组，
  用 `WheelLoop<Chassis>` 别名）· `ctlkit/VERSION` 补「PID 是 setter 式」· 诊断质量问题立项 **P33**。
  ⚠ 仍然**没有自动化**：下次上游再破就会重演 —— ① 靠人记得跑，这条清单是唯一的机制。
  一节（含"库内绿 ≠ 库外能编"的逐条动作 + 锚点比对）。① 仍需每次迁移后人工跑一次。
- **优先级**：🟡 中（每次上游迁移都会踩）
