---
class: log
generated: false
---
> **类：A 日志（append-only）** —— 交接快照，**带日期，不代表当前状态**。
> 当前状态见 [`../TODO.md`](../TODO.md)；设计见 [`../ARCHITECTURE.md`](../ARCHITECTURE.md)。
> 冻结/交接后只许追加，不许回头改。

# HANDOFF.md — 交接文档（2026-09-15）

> **用途**：给下一个 agent / 未来的自己零上下文接手。
> **开工前必读顺序**：本文件 → [`../../AGENTS.md`](../../AGENTS.md) → [`../README.md`](../README.md) →
> [`../ARCHITECTURE.md`](../ARCHITECTURE.md)。**冲突时以 `AGENTS.md` 与文档体系为准。**

---

## 1. 项目一句话

**lunokhod = 嵌入式底盘控制系统，形态是「库集合」**（不是应用、不是固件）。
C++17、每个组件零外部依赖、header-only 优先、能直接拖进 STM32 工程。

**依赖图（单向、无环、汇于最底层）**：

```
contracts  ←  kinematics
           ←  odometry
           ←  twist_acc_limiter
wheel      （谁都不依赖）
```

**第一个真实产品目标**：麦轮 2.5D 底盘底层固件（**另开仓库**，见 §10）。

---

## 2. ⚠️ 铁律（违反 = 返工）

### 2.1 角色按**文件**划分，不按重要程度

| 目录 | 谁写 | 说明 |
|---|---|---|
| `inc/` `src/` `examples/` | **用户手写** | **AI 禁止直接修改**；只给伪代码 / 参考实现 / 复查 |
| `test/` | **AI 写** | AI 是出题人，用户是答题人；见 §2.2 oracle 规则 |
| `docs/` | AI 可写 | 按四类规则（见 §4） |
| `CMakeLists.txt` · `.github/` · `scripts/` | AI 可写 | |

**用户是 C++ 学习者，正在「古法编程」**（不依赖 AI 独立写代码）。
所有建议必须解释「为什么」、给出客观标准，让用户自己动手改。

### 2.2 oracle 规则（金标不许自造）

测试里每个「期望值」都要能回答：**它从哪来？** 借的优先级：

```
① 真值     —— 实测 / 第三方库（numpy·scipy·sympy）/ 论文例题 / 参考实现
② 性质     —— 群性质·互逆·往返一致·线性·守恒·极限·收敛阶
③ 独立复算 —— 另一条数学路径的闭式解
④ 独立实现 —— 另一种语言/工具重算同一件事
```

**借不到就上报，不许自己拍一个数。** 三条硬约束：

1. 期望值**不得**由被测对象计算
2. 测试的**输入**也不得由被测对象生成
3. 容差必须**推导**（实测累积误差 × 安全系数），不许手拍 `1e-5`

### 2.3 库只交事实，不做决定（2026-09-15 新增，`AGENTS.md` §5.3）

> **接口里出现「该信谁 / 该怎么做」就是越界。**
> 判据不是「有阈值就越界」—— `twist_acc_limiter` 的 `limit_out_` 是它功能本身。

**范本**：`foucault` 的 `observe_heading(value, trust)` —— **信任度由调用者给**。
**推论**：**跨库只传标量**。禁区 = 让 lunokhod 的任何库 `include` foucault 的头，或反过来。

### 2.4 git

- **commit / push 由用户主导** —— AI 默认不做（用户明确授权时才做）
- 消息用**用户看得懂的中文**（讲"改了什么、为什么"）
- **push 前必须先在本地跑 `scripts/ci_local.py`**
- **任何 git 操作前先 `git status`**
- 文件移动用 `git mv`

---

## 3. 当前状态（2026-09-15）

```
lunokhod  main  e60008d  ·  与远端同步  ·  工作区干净
门禁 ✅（豁免 0 条）  ·  本地 CI 5 个 job 全绿  ·  聚合测试 8/8
（受管文档清单见 `scripts/check_docs.py --list` —— **不在这里抄数字，会自指**）
```

| 组件 | target | 类型 | 依赖 | 测试数 |
|---|---|---|---|---|
| `contracts` | `contracts` | INTERFACE | — | 0（纯类型，无测试） |
| `kinematics` | `kinematics` | INTERFACE | `contracts` | 3 |
| `odometry` | `odometry` | INTERFACE | `contracts` | 1 |
| `control/wheel` | `wheel` | STATIC | — | 2 |
| `control/twist_acc_limiter` | `twist_acc_limiter` | STATIC | `contracts` | 5（自身 2 + 引导进来的 kinematics 3） |

**5 个组件在 2026-09-15 定型**（此前是 3 个：`contracts` 从 `kinematics` 提为最底层库，
`odometry` 从 `kinematics/inc/` 独立成库 —— 只搬不改，7/9 个文件逐字节原样）。

---

## 4. 文档体系（AI 必须遵守）

### 四类（按**变更频率**分，不是按主题）

| 类 | 什么时候变 | 住哪 |
|---|---|---|
| `fact`（事实） | 随代码变 | 组件目录 / `docs/` |
| `log`（日志） | **只增不改**，带日期 | 组件级 `docs/log/` |
| `status`（状态） | 随进度变 | `docs/TODO.md` / 组件 `IMPL.md` |
| `work`（施工单） | 用完即弃 | `docs/`；验收后进 `trash/` |

### 六条硬规则（`../README.md` §6）

```
① 一条事实只写一处
② 日志只增不改
③ 状态不手写（脚本生成）
④ 文档里禁止出现行号；禁止出现"当前状态"
⑤ 新建文档前必须回答：它的【变更频率】和现有哪个文件不同？答不出 → 追加，不许新建
⑥ 改动触及事实时，必须在【同一次改动内】更新那一处
```

### 本轮踩过的两个坑（AI 常犯）

- **跨仓引用不许写成 markdown 链接** —— CI 只 clone 一个仓，链接在所有自动化环境里必然断。
  用**裸路径**（`~/Develop/Workspace/fw_poc/`）或**裸名字**（`foucault`）。
- **给机器看的规则，写文档时要预演机器怎么读它** ——
  我在 `docs/README.md` 里写「别用 `[...]` `(...)` 链接语法」当**反例**，门禁把**这个反例本身**
  当成链接去检查了（红）。**写反例要绕开门禁的正则模式。**

---

## 5. 工具（★ 接手后第一件事：会用这两个）

### `scripts/check_docs.py` —— 文档门禁

```bash
scripts/check_docs.py           # 查（必过）
scripts/check_docs.py --list    # 列出受管文件与它们的类
scripts/check_docs.py --why     # 打印规则 R1~R6 与依据
```

**规则**：R1 front-matter · R2 类与位置相符 · R3 禁止行号 · R4 本地链接必须存在 ·
R5 引用的仓库内文件必须存在 · R6 fact 不许是孤儿。

**已知局限**（⚠ 别被它骗）：
**R5 只看文件名不看目录** —— `basenames` 兜底导致路径写错也可能放行
（拆库时 `kinematics/inc/odometry.hpp` 已不存在，却因 `odometry/inc/odometry.hpp` 存在而静默通过）。

### `scripts/ci_local.py` —— 本地跑 CI

```bash
scripts/ci_local.py            # 跑全部 job（用工作区当前状态）
scripts/ci_local.py --clean    # 用 git HEAD 全新建 clone（查"有东西没提交"）
scripts/ci_local.py --list     # 列出有哪些 job
scripts/ci_local.py --job X    # 只跑一个
scripts/ci_local.py -v         # 失败时打印完整输出
```

**已知局限**：只支持 `matrix.<key> = [列表]` 的笛卡尔积；看到 `include:`/`exclude:` 会跳过
（所以 `ci.yml` 的 `standalone` 用简单列表 + `case` 查表，别改成 `include`）。

### 三种用法（每次改库都要全验）

```bash
# ① 聚合构建 + 全量测试（应 8/8）
cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure

# ② 每个库单独构建（contracts=0 / kinematics=3 / odometry=1 / wheel=2 / tal=5）
cmake -S <组件目录> -B build && ctest --test-dir build

# ③ 被消费时进入库模式、零泄漏（见 ci.yml 的 consumer-mode job）
```

---

## 6. 关键决策速查

> 详细记录见 [`../../AGENTS.md`](../../AGENTS.md) · [`../TODO.md`](../TODO.md) · 组件级 `docs/*_DESIGN.md`。

### 6.1 库的职责边界（2026-09-15 确立）

| 组件 | 交出的事实 | 不做的决定 |
|---|---|---|
| `odometry` | `SampleSink`：每帧 `cmd_` / `twist_` / `ws_` / `tick_` / `dt_` | 不做判定（红线：「出数据不做判定」） |
| `twist_acc_limiter` | `LimitResult` 三个饱和标志 | 不决定「被限了要不要报警」 |
| `wheel` | `get_speed()` | 不做故障判定 |
| `foucault`（另一仓） | `rejected_count()`；`observe_heading(value, trust)` | 不决定「该信多少」 |

### 6.2 「打滑检测」已重新定案（P19）—— ⚠ 别按旧理解开工

- **它是 FDI 的残差监测，不是融合。** 融合 = 多源合成估计；它是多源对比找不一致。
- **比的是瞬时量**（`FK(轮速).wz − gyro_z`），**不是位姿/角度** ——
  积分是低通，会把要抓的短时残差抹平；陀螺零偏还会积成漂移造假残差。
- **输出是权重，不是 `is_slipping` 旗标。**
- **L0（`cmd` vs `twist`）检测不到整车打滑** —— 轮子打滑时编码器跟着转。
  它实际是**执行器故障 / 轮间一致性**。
- **成熟做法是融合器的门限参数，不是模块**：见证 `reference/robot_localization`
  的 `checkMahalanobisThreshold` + 每 topic 的 `*_rejection_threshold`。
- **归属**：库里只到「暴露可观测量」为止；残差生成器与门限在**库外**。

### 6.3 底盘覆盖（`kinematics/docs/DESIGN.md`）

| 底盘 | 现状 | 关键理由 |
|---|---|---|
| 差速 2WD/4WD · 麦轮 4WD · 全向 N 轮 | ✅ 已实现 | 可达 Twist 子空间上**逆映射处处可逆** |
| **履带 / skid-steer** | ✅ **数学上已覆盖** | J 矩阵与差速**相同**；差别只在打滑（属感知） |
| **Ackermann** | ❌ 刻意不做 | **分界不是「不能横移」**（差速也不能，且契约里明确接受）—— 是**不能原地转** + `vx→0` 奇点 → 与「互逆」验收锚点冲突 |
| **Swerve** | ⬜ 待复核（P21） | 排除理由**是错的**（`THEORY.md` §2.4 的公式就是瞬时映射）。真障碍是 **`WheelSpeeds` 一维装不下 `(角度, 速度)`** |

> **口诀**：**Swerve 是「契约太窄」，Ackermann 是「抽象不成立」。** 前者值得做，后者不值得。

---

## 7. ★ 下一步任务：装配层（`chassis_loop`）

**施工单：[`../WORK_CHASSIS_LOOP.md`](../WORK_CHASSIS_LOOP.md)**（含完整接口契约）。
> ⚠ 别在文档里写「某文件 N 行」或「受管 N 份」—— **自指的计数必然错**
> （写这份 HANDOFF 本身就让受管数从 22 变成 23）。要数字就指向活源。
**4 条决策已定案**（用户 2026-09-15 按推荐拍板）：

| # | 结论 |
|---|---|
| 1 | 名字 **`chassis_loop` / `ChassisLoop`** |
| 2 | 位置 **仓库根级** |
| 3 | **构造 4 个 `Wheel`，按 `WheelSpeeds.count_` 使用**（差速只用到前 2 个） |
| 4 | **不做** `set_correction(δcmd)` |

### 它是什么

**把四块积木接到同一个时间基上，按固定顺序跑一遍。** 不生产数据，只搬运和排列。

```
set_cmd(Twist) → 限幅 → 逆解 → N×Wheel → 正解 → 里程计 → pose / yaw_ref
```

### 为什么做（证据）

同一条装配链**手写了两遍**（`~/Develop/Workspace/fw_poc/` 的 `main.cpp` 110 行、
`~/Develop/Workspace/KND_Trial/firmware/` 的 `app.cpp` 68 行）。
链里有易错知识（`tick` 顺序、`dt` 用实测值、融合钩子在里程计之后），**每重写一遍就有一次写错的机会**。

### 范围（钉死）

**只做下行链 + 里程计。**

**判据**：**只吃「已算好的量」（`Twist` / `dt` / `now`），不碰 IO。**
一旦它需要知道「命令从哪来」「IMU 在哪读」→ 那不是装配层，是应用层。

**不做姿态融合** —— 碰了 lunokhod 就要 `include` foucault 的头 ✗。
融合由使用者接一根线：`ahrs.observe_heading(loop.yaw_odo())`。

**接口要点**：`void tick(float dt, uint32_t now)` —— 装配层**不许自己发明时间基**。

### 落地顺序（施工单 §9）

```
① ✅ 已定案
② ⬜ 用户手写：chassis_loop/inc/chassis_loop.hpp + src/chassis_loop.cpp + CMakeLists.txt
③ ⬜ AI 写：chassis_loop/test/test_chassis_loop.cpp
④    三档编译 + 全量 ctest + 文档门禁 + ci_local
⑤    改 KND_Trial firmware 的 app.cpp 用本组件 → 仿真输出【逐位比对】
⑥    施工单进 trash/
```

### 验收（可机器判）

**把 KND_Trial 固件里的 `app.cpp` 换成用本组件，仿真输出必须逐位不变**：

```
改造前：最终: pose=(0.3425, 0.1085) yaw_odo=0.2574 (0.04 圈) | imu 拒绝样本=0
        四轮最终转速: 0.52 1.40 1.04 0.88
```

### 测试的 oracle（⚠ 别用循环尺子）

装配层是**接线层**，**不能拿它自己的输出算期望值**。可用的三条外部 oracle：

| oracle | 内容 |
|---|---|
| **金标** ⭐ | `fw_poc` 已跑通的仿真输出（**另一份代码**算出来的） |
| **手算锚点** | 给定几何 + 阶跃命令 → 稳态轮速由手算有理数得出（照 `test_kinematics` 的 `50.0f/3.0f`） |
| **性质** | `\|Δtwist\| ≤ acc·dt` 逐拍；`twist() == forward_kinematics(wheel_speed(i))` |

**外加两条只有装配层才有的不变式**：**调用顺序**（用可记录顺序的假 IO）·
**`tick` 不发明时间基**（`odometry` 收到的必须就是调用方给的 `now`）。

### ⚠ 一个必须记住的坑

**别在构造函数里写死配置** —— `foucault` 的 `Estimator` 就是这么写的（`solver_(make_mahony_config(dim))`），
于是 `Estimator<EKF>` 编不过，那个模板参数成了**装饰**。配置必须从 `ChassisLoopConfig` 进来。

---

## 8. 其他在办 / 已知问题（都不阻塞装配层）

| # | 事项 | 状态 |
|---|---|---|
| **P19** | 一致性残差监测 | ⬜ 被「缺标注数据」卡住；采数据的入口正是装配层的只读口 |
| **P21** | Swerve 复核 | 🟢 低（要扩输出契约） |
| **P3** | `wheel` 与 `twist_acc_limiter` 各有一份 Ramp 算法（DRY） | ⬜ 待决策 |
| **P5** | `kinematics/docs/` 三份文档的去留 | ⬜ 待拍板 |
| **P11** | PID 默认语义（与定案冲突） | ⬜ 待决策 |
| **P12** | 防御校验（`OmniDrive` 构造无参数校验，`wn>6` 越界写） | ⬜ 未开工 |
| **P2** | ~~IMU 姿态解算系统~~ | ✅ **归属已修正 —— 它就是 `foucault`**，不该挂在 lunokhod |
| **门禁 R5 盲点** | 只看文件名不看目录 | ⬜ 已记，未改脚本（改要同步 5 份副本） |

---

## 9. 仓外依赖（做固件/联调时才需要）

| 位置 | 是什么 | 状态 |
|---|---|---|
| `~/Develop/Workspace/foucault/` | 姿态解算库（独立仓，public） | 6 轴 Mahony 现役；5/5 测试绿 |
| `~/Develop/Workspace/fw_poc/` | 最早那份 PC 装配 PoC（**未纳管 git**） | ⚠️ 已被 KND_Trial 取代，只当金标参考 |
| `~/Develop/Workspace/KND_Trial/` | 底盘固件工程（**未纳管 git**） | CubeMX 工程 + `firmware/`（装配层 PoC）+ `sim/`；PC 仿真跑通 |
| `~/Develop/Workspace/cyclotron/` | FOC 项目（冻结，文档卫生已做） | 未提交 11 项 · 无远端 |

**`foucault` 的裸 float 融合钩子**是跨库交互的**范本**：

```cpp
void observe_heading(float heading_rad, float trust = 1.0f);
//                  ↑ 一个标量     ↑ 信任度由【调用者】给
```

**foucault 未提交 4 项**（H750 型号更正那批）· **cyclotron 未提交 11 项** · **cyclotron/foc 未提交 15 项**。

---

## 10. 交接说明（2026-09-15）

**本轮（2026-09-15）发生了什么**：

1. **五组件拆分** —— `contracts` 提为最底层库、`odometry` 从 `kinematics` 独立成库（**只搬不改**，
   9 个文件里 7 个逐字节原样）。动机：`odometry` 只依赖 `contracts` 却被 kinematics 的代码地图
   和错误账本"代管"；`twist_acc_limiter` 声明依赖 `kinematics` 只为拿一个 `Twist` 类型
   —— **一条语义不实的边**。
2. **本轮讨论的 7 处方案级修正**全部固化进文档（提交 `e60008d`）。
3. **装配层立项**并出施工单，4 条决策定案。

**接手建议**：

- 先跑一遍 `python3 scripts/ci_local.py --clean` 确认基线（应 5 个 job 全绿）
- 然后读 §7 的施工单，问用户「② 您手写还是我先出参考实现？」
  （先例：`WORK_ODOMETRY.md` 那次是 **AI 出参考实现 + 测试、用户手敲 `inc/`**）
- **动任何 `inc/` `src/` `examples/` 之前先问**（铁律 §2.1）

---

## 追加（2026-09-16）：装配层施工单已移入组件目录

- `docs/WORK_CHASSIS_LOOP.md` → **`chassis_loop/docs/WORK_CHASSIS_LOOP.md`**
  （理由：`chassis_loop` 是根级独立库，**组件独享的文档跟组件走** ——
  与 `kinematics/docs/DEV_GUIDE.md`、`odometry/docs/ODOMETRY_DESIGN.md` 同类）
- **⚠ 所以 §7 里那行施工单链接已失效** —— 请按上面的新路径找。
  A 类日志**只增不改**，所以没回头改那一行；门禁的 R4（链接必须存在）
  已按 R5 的**同一条理由**对 log 类豁免：文件搬走后，旧日志里的路径就是历史快照。
- §7 的内容本身**仍然有效**（范围 / 验收锚点 / oracle 三条都没变）。
  新增的只有接口层决策 **D1~D8**（含 **D6：底盘实例从外面注入**），见施工单 §12。
- 顺带：`TODO.md` **P16 已关闭**（`Odometry::reset(Pose)` 早就有，属过期条目）；
  新增 **P22**（装配层 `reset()` 语义待设计）。
- **二次追加（同日）**：装配层的**长期事实已拆出为 `chassis_loop/docs/DESIGN.md`**
  （契约 / 决策 D1~D9 / 职责与「五个不做」/ 执行顺序 / 只读口 / 已知边界 F1~F3 / 验收锚点）。
  `chassis_loop/docs/WORK_CHASSIS_LOOP.md` 现在只剩「参考实现 + CMakeLists + 测试 + 判别力审计 + 手敲顺序」，
  验收后进 `trash/`。**上面 §7 说的"含完整接口契约"以 `DESIGN.md` 为准。**
- **三次追加（2026-09-16，同日）**：装配层**容量 4 → 6**（对齐契约 `WheelSpeeds.values_[6]`，
  `chassis_loop/docs/DESIGN.md` §8.2 新增 **D10**）。原因是一处真 UB：
  `ChassisLoop<OmniDrive>`（wn=6）在 `wheels_[i]` 上越界 → UBSan `index 4 out of bounds` + ASan `SEGV`。
  **P24 已关闭**；同时补了**边界 N 测试**（三轮 / 六轮），并验证「容量退回 4 → 新用例在 ASan 下必红」。
  已复验：四档编译零告警 · 聚合 9/9 · 9 个变异全红 · **KND_Trial 仿真输出仍逐位相同**（容量变更对 4 轮无影响）。
  > 教训（已入 `chassis_loop/AGENTS.md` 账本）：**"支持 N" 的组件必须测边界 N**；
  > **装配层容量必须 ≥ 它接受的契约容量**。
- **四次追加（2026-09-16，同日）**：本轮是**概念澄清 + 需求归口**，**无代码改动**。
  - **`SetPwmFn` 定位定案**：它**不是「碰了硬件」，而是「命名绑定了实现」**。
    `SetPwmFn` 是**注入的回调** —— 库连「这是 PWM」都不知道（对面可能是 TIM 寄存器 / PC 模型 / 日志文件）。
    **硬件的边界是「调用 vs 被调用」，不是「数字 vs 硬件」**；真·PWM 实现（`hal::set_pwm`）叫得对，
    错的只是把它写进**库的接口**（库的接口属于所有调用方）。
    → 改名方案 **`SetEffortFn`**，对称性最强：`MeasureSpeedFn`→`speed_cur_`→`get_speed()`
    ↔ `SetEffortFn`→`effort_`→`effort()`。
  - **PID 算法需求归口到仓外**：`~/Develop/Workspace/pid/`（PID 对比归档 + 优化考量清单，**未纳管 git**）。
    本轮新增条目 **D3（饱和/限幅状态上报）** 已入其 `docs/optimization_considerations.md`
    （改前已备份到 `/tmp/pid_backup_<ts>/`）。⚠ 落地要**两份活副本同改**：
    `cyclotron/foc/inc/foc/algo/pid.hpp`（该仓声明「以此为准」）+ `lunokhod/control/wheel/inc/pid.hpp`
    —— 两者 `pid.cpp` **逐字节相同**，只差 namespace / include。
  - **TODO 新增**：**P25**（装配层接出 `LimitResult` 的三个饱和标志 —— 现在被 `.out_` 顺手扔了）·
    **P26**（轮级 `effort()` 暴露 + `SetPwmFn`→`SetEffortFn`，建议与 P11/P23 一起进 **wheel v0.2.0**）。
    **P3 扩大**为跨仓同源副本 · **P11 增「pid 仓 D3 硬前置」身份**（`limit_out_=0` 会让饱和标志恒 true）·
    **P19 补 §九** · **P23 并入 P26**。
  - **P19 的两个实质进展**（原先是「被采数据卡住」）：
    ① **能分类打滑/堵转的最小数据集 = 三元组** `wheel_target(i)` / `wheel_speed(i)` / `wheel_effort(i)`
    —— 缺第三个就分不开（**堵转与空转在转速上是同一个观测**，只有努力度分得开）；
    ② **采数据零改动可做**：`SampleSink` 在 `odom_.update()` 内**同步调用**，而 `odom_.update()` 是
    `tick()` 的**最后一步** → **在 sink 回调里读装配层只读口，拿到的就是同一拍的值**（不需缓冲 / 时间戳对齐）。
  - **另一条实质结论**：**残差的第一消费者是「标定」，不是「打滑」** ——
    标定误差 = **长期一致的系统性残差**，打滑 = **瞬时残差**；共用采样，**判定方法完全不同**（见 P19 §九.5）。
  - ⚠ **门禁坑复现（写文档时踩到）**：把一个仓外路径写进反引号
    （`~/Develop/Workspace/pid/docs/xxx.md`），R5 把其尾段 `docs/xxx.md` 当成**本仓相对路径**报红。
    **解**：跨仓引用用**裸名字**（`optimization_considerations.md`），路径单独写 —— 与本文档 §6「跨仓引用」同一条规矩。
