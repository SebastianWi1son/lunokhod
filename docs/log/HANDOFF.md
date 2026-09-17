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
- **四次追加（2026-09-17）**：**装配层的"通用框架"已落地** —— 中间那一格抽成
  **执行器组**（`chassis_loop/inc/wheel_set.hpp`，契约 `DESIGN.md` §5.2 / 决策 **D11**），
  装配层从此**不认识底盘与轮子**。`measure` 收设定值（**D12**）。
  验收：四档零告警 · 聚合 9/9 · 12 变异全红 · **KND 仿真输出逐位相同**（md5 `9ab64f43…`）。
  **下游 KND_Trial 按用户决定未动**（它因 ctlkit 迁移编不过，见 `../TODO.md` **P30**）。
  另：算法原语已 vendor 上游 `ctlkit` v0.1.1（`third_party/ctlkit/`），`SetPwmFn` → `SetEffortFn`（P26 起步）。
- **五次追加（2026-09-17）**：**外部 agent 的「冻结前 13 条」已逐条核实**（10 条属实 / 3 条不准），
  更正进了 `../TODO.md`；冻结前的收尾顺序如下（**代码你手敲，文档我善后**）：

  | 批次 | 内容 | 谁 |
  |---|---|---|
  | **1** | **P25 限幅饱和标志 + P26 effort/命名中性化 + P23 显式 cast**（同一个文件，一次编完） | **你** |
  | **2** | **P12 `OmniDrive` 构造校验**（`wn > 6` 会静默踩内存 —— 建议与批次 1 同批） | **你** |
  | **3** | 文档全部：`DESIGN §5 残留`·`§5.1/§5.2 顺序`·`Swerve 引用 P26→P28`·`ctlkit 下游核对清单`·`P31 接入指南`（已做） | 我 |
  | **4** | **KND_Trial 两处断点**（扁平 PIDConfig 字段 + 接缝构造签名） | 你 / 下游 |
  | 推后 | `P29` 命名空间（破坏性，与 P26 同批发版）· `P22` reset 语义（等 G7）· `P19`/G3 残差与融合 | — |

  **三条更正**（原清单错的地方，别照抄）：① P23 的 `-Wconversion` **只有 1 处真告警**（另一处是常量 `0.0f`）；
  ② 原清单 #5「执行器组契约要加口」**不成立** —— `wheel_speed(i)`/`wheel_target(i)` 读的是装配层自己的缓存，
  真身是 **P26 的子问题**「要不要 `wheel_effort(i)`」，结论：**先不加**；③ P29 的全局名是 **15 个**不是 8 个。
  新增 **P31**（下游接入指南）→ 已落地 `../INTEGRATION.md`（示例代码真编译真跑过）。
- **六次追加（2026-09-17 晚）**：**P29 命名空间 + P26 加口/改名 + P23 显式 cast 全部落地**（AI 做 ——
  用户判定这三条属"加作用域 / 改名"类机械活）。要点：
  · `wheel` 的类型收进 **`namespace wheel`** → 15 个全局名消失；消费方写 `wheel::PID` 或自己 `using`。
    同批把 4 个转发头的**过期注释**改了（原文还写着"保留全局名"）。
  · 执行器组契约加**第 5 个方法** `int16_t effort(uint8_t i) const`（决策 **D13**）→
    `wheel::Wheel::effort()` / `WheelSet::effort(i)` / `ChassisLoop::wheel_effort(i)`。
  · `float→int16_t` 全部显式 `static_cast`（**`-Wconversion` 严格档零告警**）。
  验收：聚合 9/9 · 四档零告警 · **15/15 变异变红**（含 3 条专测新口）· 消费模式零泄漏 ·
  `docs/INTEGRATION.md` 的示例**走 CMake 链真编真跑** · **KND 仿真逐位不变**（md5 `9ab64f43…`）。
  **下一步：只剩 `P25` + `P12` 要你手敲 → 施工单 `docs/WORK_FREEZE.md`。**
- **七次追加（2026-09-17 深夜）**：**P32 全仓命名空间政策落地**（用户拍板 **A 案**：根 `lunokhod::` +
  子命名空间 = 组件名；**跨库数据 `Twist`/`WheelSpeeds`/`Pose` 放根**）。起因：用户发现 P29 只圈了
  `wheel` 一个组件，而 **foucault 有明文决策 F10**（`foucault::` + `math`/`solver`/`measure`）——
  "一半 `wheel::`、一半全局"是最不一致的状态。
  · 规则转正到 **`AGENTS.md` §3.1**（唯一来源）；`ARCHITECTURE.md` §0.1 分层表加了「命名空间」列。
  · 10 个库文件包命名空间；跨组件引用写兄弟命名空间（`odometry::SampleSink`）；消费方（测试/示例/工具/
    CI 示例/接入指南）跟上；其余组件文档加"本文省略 `lunokhod::` 前缀"的说明。
  · 验收：聚合 9/9 · 四档零告警 · **全局命名空间已清空**（探针：全局再定义 `Twist`/`Odometry`/`Wheel`/
    `ChassisLoop` 等 8 个同名类型可共存）· **KND 仿真输出逐位不变**（md5 `9ab64f43…`）。
  **下一步不变：`P25` + `P12` 两条要你手敲 —— 施工单 `docs/WORK_FREEZE.md` 已按新命名空间更新。**
- **八次追加（2026-09-17 深夜）**：**P12 与 P25 收口**。
  · **P25**（限幅饱和标志）：用户手敲，已核 —— `lim_res_` + `limit_result()`，`t_cmd_final_ = lim_res_.out_`
    （第一版曾**多留一行** → `limit()` 每拍被调两次 → 加速度上限翻倍，测试红 39 条，已修）。
  · **P12**（`OmniDrive` 轮数）：**三轮迭代才到位** —— 夹住（洗白数据）→ 判无效（靠自觉）→
    **模板参数 `OmniDrive<N>` + `static_assert`**（非法轮数**写不出来**）。用户授权 AI 直接改 `inc/`。
    新增 **CMake 反例编译测试**（`test/compile_fail/omni_over_capacity.cpp` + `try_compile`）：
    守卫被删/放宽 → 配置阶段 FATAL_ERROR（已实测有牙齿）。调用点 8 处已同步。
  · 验收：聚合 9/9 · 严格档零告警 · 反例测试有效 · **KND 仿真输出逐位不变**（md5 `9ab64f43…`）。
  · 教训入账：`kinematics/AGENTS.md` **账本第一行**（「能用常量就别用参数」+「别指望自觉，进类型系统或 CI」）。

---

## 🧊 冻结快照（2026-09-17，本文件所在提交 = 冻结点）

**状态**：lunokhod **库本体冻结**，供下游消费（KND_Trial / fw_poc / 下一个产品）。
下一轮工作的对象是**下游（固件 + 平台层）**，不是本仓。

### 冻结点是什么

| 项 | 值 |
|---|---|
| 冻结点 | `main` @ **本文件所在提交**（`git log -1`） |
| 六个组件 | `contracts` · `kinematics` · `odometry` · `command/twist_acc_limiter` · `actuator/wheel` · `chassis_loop` |
| 命名空间 | 全仓 `lunokhod::`（`AGENTS.md` §3.1）；跨库数据 `Twist`/`Pose`/`WheelSpeeds` 在根 |
| 装配接缝 | 执行器组契约（`chassis_loop/docs/DESIGN.md` §5.2，决策 **D11/D12/D13**，**5 个方法**） |
| 验收凭据 | 聚合 `ctest` **9/9** · 四档编译零告警（Debug/Release/严格档含 `-Wconversion`/ASan+UBSan）· **变异审计全红** · **KND 仿真输出逐位不变** md5 `9ab64f43200d263c8386490ae7d70c09` · `scripts/ci_local.py --clean` 五 job 全绿 |
| 下游入口 | [`../../docs/INTEGRATION.md`](../../docs/INTEGRATION.md)（平台层两个函数 + 一份能跑通的装配 + 5 个坑） |

### 冻结时**已经知道**的开放项（都不阻塞下游，别当成缺陷）

| 编号 | 是什么 | 何时做 |
|---|---|---|
| **P27** | FOC 作为执行器组（接缝已装得下，写一个 `FocSet` 即可） | 真接 FOC 时立项 |
| **P28** | Swerve：**输出契约要一起扩**（`WheelSpeeds` 装不下「角度+速度」） | Swerve 立项时 |
| **P30** | 下游是外仓、未提交 → "库内绿、库外断"没有自动核对；`third_party/ctlkit/VERSION` 有核对清单 | 每次上游迁移 |
| **P22** | 装配层 `reset()` 语义（各积木有 `reset` 但装配层是 private） | 等固件急停/失效保护（缺口 G7） |
| **P19** | 一致性残差监测（`wz` 残差 = `FK(实测轮速).wz − gyro_z`）；原料已齐（`SampleSink`） | 融合层立项时 |
| **P12/P23/P24/P25/P26/P29/P32** | ✅ **均已闭合** | — |

### 解冻须知（下次改本仓之前）

1. **先读**：本文件 → `AGENTS.md`（规则/账本）→ `docs/ARCHITECTURE.md` §0.1（层 ↔ 命名空间）→ 目标组件的 `docs/DESIGN.md`（契约权威）。
2. **破坏性改动仍然很便宜**（下游只有两个未提交的 PoC），但**必须**：先改设计 → 同步测试与代码 → 同步文档，**同一次改动内**完成。
3. **规矩没变**：`inc/` `src/` `examples/` 用户手写（除明确授权）；`test/` AI 写；文档行号禁止（R3）；提交双段式（`docs/GIT.md`）。
4. **改完必过**：`python3 scripts/ci_local.py --clean` + `md5` 锚点比对（KND 隔离副本，见 `docs/TODO.md` P30 的核对清单）。

---

## 冻结快照补遗：外部复核（2026-09-17 同日，冻结点之前）

外部 agent 实跑 `scripts/ci_local.py` 后报了 3 条，逐条核实：

| # | 报告 | 核实结果 |
|---|---|---|
| 1 | 编译失败（`kMinWheels=3` vs 文案 `2..6` vs 测试 `OmniDrive<2>`） | **曾真实存在**：`kMinWheels` 由用户改成 3 时，文案与测试还是 AI 按 2 写的。当轮已修；**再根治为「文案不含数字 + 上下界各一个反例编译测试」**（`<2>` 与 `<7>` 都必须编不过 → 边界一被放宽，`cmake` 配置阶段就红） |
| 2 | `lim_res_;` 少 `{}` → 潜在 UB | **属实（AI 漏了）**。已修：成员 `lim_res_{}` + **`LimitResult` 加默认成员初始化器**（从类型上堵死，`limit()` 里那个 `LimitResult r;` 一并受益）。⚠ **没有测试能可靠抓住它**（未初始化读是 UB，ASan/UBSan 不覆盖，实测去掉 `{}` 仍 `ALL PASS`）→ 已记入 `chassis_loop/AGENTS.md` 账本 |
| 3 | 施工单说"待你手敲"，代码却都做了 | **P25 = 用户手敲**（第一版多留一行，AI 报、用户删）；**P12 = 用户敲两版 → AI 重写为模板版 → 用户改 `kMinWheels=3`**。施工单已归档 `trash/`，归档件里补了"谁敲的"备注 |

**新增**：`chassis_loop` 测试第 **10** 组（限幅结果暴露：首次 tick 前零值 · 阶跃后饱和标志 · 与 `cmd()` 一致）；
`kinematics/test/compile_fail/` 两个反例（上界 `omni_over_capacity` / 下界 `omni_under_minimum`）。

### 补记（同日稍晚）：`fw_poc` 退役

下游那个 PC 装配 PoC（`~/Develop/Workspace/fw_poc/`，110 行，**未纳管 git**）经核实**内容已全部并入
`KND_Trial`** —— 实测**两份手写链的仿真输出 md5 逐位一致**（`9ab64f43…`）。故它**不再是消费方**：

- 它的**输出锚点**（此前只在 `/tmp` + 一个**无法重新编译**的旧二进制里）已入仓：
  `chassis_loop/test/golden/knd_sim_anchor.txt`（溯源与复核方法见 `chassis_loop/docs/log/ACCEPTANCE.md` 附录）。
- 仓内**前瞻性引用已重指**（顶层 `CMakeLists.txt` / `README.md` / `third_party/ctlkit/VERSION` /
  `docs/TODO.md` P30 / `docs/README.md` 的"裸路径"示例）；**历史性引用原样保留**（日志只增不改）。
- **消费方只剩一个**：`KND_Trial`。上面"供下游消费（KND_Trial / fw_poc / 下一个产品）"里的 `fw_poc` 请按此理解。

### 补记二（同日）：拿真下游做**接触实测**（P30 的手工版）

在 `/tmp` 隔离副本（**真 `KND_Trial` 一行未动**）上对着冻结版 lunokhod 走了一遍完整迁移：

| 项 | 结果 |
|---|---|
| 迁移成本 | **3 个文件、约 60 行**（`app/inc/app.hpp` · `app/src/app.cpp` · `sim/src/main.cpp`） |
| 迁移内容 | ① 5 行 `using`（`lunokhod` + 4 个子命名空间）② `PIDConfig` 改 **setter 式** ③ 配置分家（`ChassisLoopConfig` 只剩限幅+里程计，PID/规划器进 `WheelSetConfig`）④ 构造注入执行器组 |
| **验收** | 迁移后 `knd_sim` 输出与 `chassis_loop/test/golden/knd_sim_anchor.txt` **逐位一致** ✓ |
| 板级固件 | 交叉编 `arm-none-eabi-g++` **过** ✓ 固件 (`KND_Trial.elf`) 也过 —— 但**固件今天并没有链 `knd_app`**（只 `stm32cubemx + knd_bsp + knd_drivers`）→ 控制链上板是下游未来的活 |
| **MCU 侧首次真验** | 冻结版全部库 + 一个固件形状的探针 TU，`-fno-exceptions -fno-rtti -Wall -Wextra -Werror`（外加 `-Wconversion -Wshadow -pedantic`）**零告警**；未定义符号只有 `__aeabi_*` / `libm` / `memcpy` + 库自身 —— **无 libstdc++、无 `__cxa_*`、无异常/RTTI** ✓；整链 `.text` ≈1.3 KB + `.bss` ≈1.9 KB |
| 下游 host 测试 | `KND_Trial/tests` **3/3 过**（板级不碰算法库，天然免疫） |

**摩擦点（按价值排序）**：
1. **`ChassisLoop` 模板参数写错时诊断质量差** —— 报错指向库内部，不指向用户那行（详见 `TODO.md` **P33**）；本轮已在接入指南 §6 补成第 6 条坑，并确认 `WheelLoop<Chassis>` 别名是最短正解
2. `WheelSet` / `WheelLoop` 住在**另一个头** `wheel_set.hpp`（`chassis_loop.hpp` 故意不 include 它 —— 依赖倒置）→ 消费方得照接入指南 include；接入指南已必读
3. `ctlkit` 的 `PIDConfig` 是 **setter 式**（`p.kp(8.0f)`），非字段式 —— 已在 `third_party/ctlkit/VERSION` 补提示
4. **文档够用**：`docs/INTEGRATION.md` 的 5 行 `using` + include 清单与本次实际所需**逐字吻合**（没多没少）

### 补记三（同日）：真下游已迁移，P30 关闭

用户授权后**直接改了真 `KND_Trial`**（此前一切都在 `/tmp` 隔离副本上做）——
按「尽量只改 lunokhod 相关内容」的要求，**只动 3 个文件**：`app/inc/app.hpp` · `app/src/app.cpp` ·
`sim/src/main.cpp`（`git status` 里正好只有这 3 个从 `A ` 变 `AM `，其余 172 个文件未碰）。

| 验收 | 结果 |
|---|---|
| `knd_sim` 输出 | 与 `chassis_loop/test/golden/knd_sim_anchor.txt` **逐位一致** ✓ |
| KND host 测试 | **3/3 过** ✓ |
| 板级固件（`arm-none-eabi`） | 交叉编译**过**，产出 2.3 MB `.elf` ✓ |
| 告警 | 零 ✓ |

**迁移三件事**（都只碰调用点）：① 5 行 `using` ② `PIDConfig` 改 setter 式 ③ 配置分家 + 构造注入
执行器组（`WheelLoop<MecanumDrive>` 别名，一行说完）。**KND 侧 `hal::set_pwm` 名字保留不动** ——
那是它平台层的语义（写 PWM），与库侧 `SetEffortFn` 的契约兼容。

**未做（有意）**：不在 KND 里 commit（该仓**尚无首个 commit**，175 个文件全在暂存区 —— 首次提交是用户的事）。
