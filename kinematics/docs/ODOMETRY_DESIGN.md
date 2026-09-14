# ODOMETRY_DESIGN.md — lunokhod/kinematics 里程计组件设计

> 状态：**v3 定稿**（2026-09-13）——O1~O9 决策点 + R1~R7 修正全部拍板，可进入实现
> 地位：里程计 = 底盘运动系统的**状态侧**核心组件（kinematics 是纯函数瞬时映射，odometry 是有状态积分器 + 数据记录源）
> 实现红线：按 `AGENT.md`，代码由作者手写，AI 只给伪代码/参考实现与复查

## 1. 需求与角色（为什么需要它——先讲清楚）

**角色锐化：Odometry = "轮子的尺子"——对"轮子转动"的唯一事实源。**

它不猜（不做状态估计/融合），它只量（测量轮子转了多少）；它输出的所有量都是条件参考：**"如果轮子无打滑、模型无误差，车应该在这"**。猜的活（融合/估计）交给消费方。

**一个数据源，三个出口（被三种视角消费）：**

```
                 ┌─────────────────────────────────────────────┐
                 │           Odometry（轮子的尺子）              │
                 │   输入: 轮速 + 指令 + tick（纯机械数据）      │
                 └──────┬──────────────┬──────────────┬─────────┘
                        │              │              │
                  ① pose(条件位姿) ② yaw_ref(独立yaw观测) ③ samples(原始记录)
                        │              │              │
                        ▼              ▼              ▼
                 导航/上层开环控制   foucault校准源   建模/打滑分析/回放
                              （抵消 yaw 漂移）
```

- ① `pose`：假设模型精确的位姿 → 上层开环导航（"如果没打滑，车在这"）
- ② `yaw_ref`：**独立于 IMU 的** yaw 观测（长期不漂）→ 融合层用
- ③ `samples`：原始轮速+指令+实际体速度+位姿+时间戳 → 建模/检测/回放

**核心需求（用户原话提炼）**：
1. **校准源**：foucault（纯 6 轴 IMU）yaw 不可观必漂 → odometry 的轮速积分 yaw **长期不漂**，作外部校准源抵消漂移
2. **纯里程记录**：采集原始里程数据 → 底盘建模（系统辨识）、打滑检测、离线回放

**为什么 odometry 不做融合（分工红线）**：
- **依赖方向**：融合要吃 IMU 数据流 + 时间戳对齐 + 噪声模型 → odometry 会从"底盘无关"变成"依赖 IMU 硬件"，破坏解耦
- **单一职责**：odometry = 纯机械数据（轮子转了多少）；融合 = 多源估计（车在哪）——尺子不当裁判
- **可测性**：odometry 锚点是纯数学（圆轨迹闭合）；融合测试需要 IMU 数据集（foucault 的回放主场）
- **复用**：融合逻辑跟姿态库（foucault）走，云台/其他项目直接复用，不绑底盘

**融合闭环（用户提出，正确且必做——但归属融合层）**：
```
里程计 yaw_ref ──→ 融合层(互补/EKF) ←── foucault IMU yaw
        ↑                                    │
        └────────── 打滑检测(IMU 参照) ←──────┘
里程计做 IMU 的 yaw 校准；IMU 做里程计的打滑检测参照——双向互补，融合层实现
```

## 2. 定位与边界

**定位**：把轮速（经 forward 正解）积分为车体位姿 `(x, y, yaw)` 的有状态组件；同时是**数据记录源**（每帧原始数据 + 位姿快照）。

**明确不做**（v1）：
- 融合算法本身（IMU+里程计融合是 foucault/未来组件的职责——odometry 只**提供**校准参考）
- 打滑**检测算法**（v1 只保证**数据形状**足够检测用；算法留 v2）
- 路径规划 / 轨迹生成
- 轮径在线标定（只留 `twist_scale_` 修正口，语义见 §4）

**归属（O1 定案）**：文件 `kinematics/inc/odometry.hpp`，**不并入 `chassis.hpp`**。
理由：kinematics 的对外身份是"纯函数瞬时映射"，`chassis.hpp` 是这一层的唯一入口；odometry 是有状态、可选、非所有用户都需要的组件，作为 opt-in 兄弟头文件单独 `#include`。相应地 `AGENT.md` 的"用户唯一入口"表述修正为"**瞬时映射唯一入口**"。

## 3. 数据流

```
Wheel 实际速度 ×4 → MecanumDrive.forward → Twist（瞬时）→ Odometry.update(twist, dt, tick, cmd, ws)
                                                              │
                                              ┌───────────────┼───────────────┐
                                              ▼               ▼               ▼
                                          Pose 位姿      yaw 校准参考    记录样本(可选写入)
                                              │               │               │
                                              ▼               ▼               ▼
                                        导航/上层      → foucault      → 数据导出/回放
```

**关键解耦**：Odometry 吃 **Twist**（瞬时体速度），不吃原始轮速 → 对底盘类型零感知。正解在外部调。
原始轮速 `ws` 只作为**记录字段**（需求③）传入，不参与积分——积分永远走 Twist。

## 4. 数学（积分模型，R6 定案：半隐式欧拉）

```
① 标定缩放：  vx' = k·vx,  vy' = k·vy,  wz' = k·wz      (k = twist_scale_)
② 先积分 yaw：yaw_new = yaw + wz'·dt
③ 再用新 yaw 旋转位移（半隐式，比显式欧拉少一阶误差）：
   x_new = x + (vx'·cos(yaw_new) − vy'·sin(yaw_new))·dt
   y_new = y + (vx'·sin(yaw_new) + vy'·cos(yaw_new))·dt
```

- 积分法：**半隐式欧拉**（先积分 yaw，再用新 yaw 算位移）。梯形法为未来升级项（O5）
- **yaw 存储**：内部**连续累计**（支持多圈）；两个出口分工固定（R2）：
  - `yaw_ref()` / `yaw_continuous()` → 连续累计值（给融合层当校准源，单调不跳变）
  - `pose().yaw_` → wrap 到 (−π, π]（给导航/上层，省得自己转）
- **`twist_scale_` 语义（R3，替代原 `wheel_radius_calib_`）**：
  本体系 Twist 的整体标定系数。数学依据：均匀轮径误差下 `vx ∝ r·ω`、`wz ∝ r·ω/b` 同步放大，
  等价于把整个 Twist 乘 k。**只覆盖均匀轮径误差；轮间差异（左右不一致）在校准/编码器层解决**——
  命名用 `twist_scale_` 而非轮径，是守住"odometry 不知道轮子存在"的底盘无关红线。
- 位置 x/y 是世界系；与"forward 输出瞬时 twist"严格区分（那是 kinematics 的活）

## 5. 接口定稿（v3）

```cpp
// ---- contracts.hpp 内：Pose 数据契约（O3 定案，跨组件输出契约归数据层）----
struct Pose {
    float x_, y_, yaw_;      // 世界系，m / rad（yaw_ 为 wrap 值）
};

// ---- odometry.hpp 内 ----
struct OdometrySample {      // R4：轮速用契约 WheelSpeeds（支持 3/4/N 轮，不硬编码 4）
    uint32_t    tick_;       // 时钟刻度（ms/us 计数，零依赖；单位由调用方约定并文档声明）
    float       dt_;         // 本帧间隔（s）
    WheelSpeeds ws_;         // 原始轮速（rad/s，实际值；打滑检测/建模用）
    Twist       cmd_;        // 指令 twist（期望值）
    Twist       twist_;      // 本帧实际体速度（标定后，用于 cmd vs 实际 的打滑信号）
    Pose        pose_;       // 积分后位姿
};

// R5：注入式 sink 带 ctx，避免全局/静态变量承载写入句柄（PC 写 CSV、嵌入式写串口）
using SampleSink = void (*)(void* ctx, const OdometrySample&);

class Odometry {
public:
    explicit Odometry(float twist_scale = 1.0f);

    void  set_sink(SampleSink sink, void* ctx = nullptr);   // 注册记录出口（null = 不记录）
    void  reset();                                          // 位姿清零（保留 sink 与 twist_scale）

    // R7：sink 已设置但 ws == nullptr 时不触发记录（宁可丢帧，不写脏数据）
    //     cmd 仅在记录时使用，不参与积分
    Pose  update(const Twist& body_vel, float dt, uint32_t tick,
                 const Twist& cmd = {}, const WheelSpeeds* ws = nullptr);

    Pose  pose() const;             // yaw_ wrap 到 (−π, π]
    float yaw_continuous() const;   // 连续累计 yaw（多圈单调）
    float yaw_ref() const;          // = yaw_continuous()，融合层校准源出口

private:
    float       x_ = 0.0f, y_ = 0.0f, yaw_ = 0.0f;   // yaw_ 连续累计
    float       twist_scale_ = 1.0f;
    SampleSink  sink_ = nullptr;
    void*       sink_ctx_ = nullptr;
};
```

- 单位：m / rad / s（中性化）；状态私有 + 只读查询
- **记录职责边界**：Odometry 只负责**组装样本 + 触发 sink**，不持有缓冲/文件（消费方自决：PC 建模侧写 CSV/回放文件，嵌入式侧接串口/Flash）
- `update` 返回 `Pose`（当帧结果），`pose()` 供异步查询——两者一致

## 6. 决策记录（O1~O9，全部定案 ✅）

| # | 决策点 | 定案 | 说明 |
|---|---|---|---|
| **O1** | 归属 | ✅ kinematics 库内 `inc/odometry.hpp`，**不进 `chassis.hpp`** | 见 §2；R1 修正了原案"chassis 聚合加入" |
| **O2** | 输入形态 | ✅ `update(Twist, dt, tick, cmd, ws)` | 底盘无关；`ws` 只做记录字段，不参与积分 |
| **O3** | Pose 归属 | ✅ 进 `contracts.hpp` | 跨组件输出契约归数据层 |
| **O4** | yaw 存储 | ✅ 内部连续累计 + 双出口分工 | `yaw_ref()` 连续 / `pose()` wrap，见 §4·R2 |
| O5 | 积分法 | ✅ 半隐式欧拉 v1（梯形留 v2） | R6 钉死顺序：先 yaw 后位移 |
| O6 | 修正口 | ✅ `twist_scale_` | R3 改名（原 `wheel_radius_calib_`），守底盘无关红线 |
| O7 | 锚点测试 | ✅ 见 §7（9 条） | #6/#7/#8 依赖 O8/O9 实现 |
| **O8** | 记录形态 | ✅ 注入式 sink 回调（**带 ctx**） | R5：`void(*)(void*, const OdometrySample&)` + `set_sink(fn, ctx)` |
| **O9** | 样本内容 | ✅ 全量：`tick + dt + ws + cmd + twist + pose` | R4：`ws` 用 `WheelSpeeds`；新增实际 `twist_`（cmd vs 实际的打滑信号） |

## 7. 修正记录（R1~R7，相对 v2 草案的变更）

| # | 变更 | 理由 |
|---|---|---|
| R1 | odometry **不进 `chassis.hpp`**；AGENT.md 表述改"瞬时映射唯一入口" | 有状态组件污染"纯函数瞬时映射"的对外身份 |
| R2 | `yaw_ref()`=连续、`pose().yaw_`=wrap，不用布尔参数开关 | 消费方需求相反，用两个明确出口代替运行时开关 |
| R3 | `wheel_radius_calib_` → `twist_scale_` | 均匀轮径误差 ≡ 整体 Twist 缩放；命名不得暗示 odometry 知道轮子 |
| R4 | 样本 `float ws_[4]` → `WheelSpeeds ws_`；新增 `Twist twist_` | 支持 3/4/N 轮；打滑检测需要"期望 vs 实际"两个 Twist |
| R5 | sink 加 `void* ctx` | 函数指针无法捕获状态；避免消费方被迫用全局变量 |
| R6 | 明确半隐式欧拉（先积分 yaw，再用新 yaw 算位移） | 避免实现时两种写法各写一半、锚点对不上 |
| R7 | `sink` 已设但 `ws==nullptr` → 不触发记录 | 宁可丢帧也不写脏数据（与 P12 防御校验同精神） |

## 8. 测试锚点（9 条）

1. **恒速直行**：vx 恒、wz=0 → x 线性增长，yaw 恒 0
2. **圆轨迹闭合**：vx/wz 恒 → 轨迹半径 ≈ vx/wz（多点采样圆拟合，容差）
3. **yaw 回绕**：wz 大、多圈 → 内部连续累计不跳变；`pose().yaw_` 落在 (−π, π] 且连续
4. **零速不动**：Twist 全零 → Pose 不变
5. **世界系旋转**：初始 yaw=π/2、纯 vy（麦轮横移）→ 世界系沿 x 移动
6. **记录触发**：注入 sink → 每帧收到样本，字段与输入一致（`tick` 单调、ws/cmd/twist/pose 正确）
7. **校准参考**：`yaw_ref()` 与 `yaw_continuous()` 一致；多圈下单调不跳
8. **R7 防御**：sink 已设、`ws=nullptr` → sink 不触发（计数 0）；`ws` 传值 → 正常触发
9. **标定系数**：`twist_scale=2` → 同输入下位移/转角 ×2；`twist_scale=1` 与默认构造等价

## 9. 实现顺序（建议，供手写参考）

1. `contracts.hpp` 加 `Pose`（不改动已有 Twist/WheelSpeeds，零破坏）
2. 新建 `inc/odometry.hpp`：状态 + `reset()` + `update()` 积分 + `pose()`/`yaw_continuous()`/`yaw_ref()`
   - 先不加 sink，把 §8 锚点 1~5、9 跑绿
3. 加 `SampleSink` + `set_sink` + 样本组装（锚点 6）
4. 加 R7 空指针守卫（锚点 8）
5. CMake：`test_odometry` target（镜像 `test_kinematics`，`-Wall -Wextra -Werror`）
6. 跑完后同步 `docs/IMPLEMENTATION_TRUTH.md`（该文档的维护规则：改代码必须同步）

## 10. 未来拓展（记账）

- 打滑**检测算法**（样本已含数据：cmd vs 实际 twist、轮间一致性、与 IMU 加速度对照）
- 梯形积分升级 / 轮径在线估计 / 里程计协方差（EKF 融合输入）
- 记录导出工具：CSV/回放（对齐 foucault host/replay 模式）
- IMU 辅助位置（foucault 加速度短时修正位置，反向互补）

## 11. 分工与参考（融合在融合层，不在 odometry）

| 组件 | 职责 | 输出 | 消费方 |
|---|---|---|---|
| **Odometry** | 轮速→位姿积分 + 记录 | pose / yaw_ref / samples | 上层 / 融合层 / 建模 |
| **foucault** | IMU→姿态（四元数） | yaw/pitch/roll | 上层 / 融合层 |
| **融合层**（未来组件，可挂 foucault estimator） | 互补/EKF：yaw 互补 + 打滑降权 | 融合位姿 | 导航/定位 |

**融合层实现逻辑（用户闭环落地）**：
- 低频：里程计 yaw_ref 压 IMU 漂移（yaw 校准）
- 高频：IMU 平滑短时抖动
- 打滑：IMU 加速度/角速度与轮速一致性异常 → 里程计协方差调大/降权（不硬丢弃，EKF 调权思想）

**市场参考（已 clone `reference/robot_localization`，36M，已 gitignore 不入库）**：
- ROS 生态最主流轮式里程计+IMU 融合包（EKF/UKF，15 维状态含 gyro bias，two_d_mode 2D 平面）
- 核心可读：`src/ekf.cpp`、`src/filter_base.cpp`（滤波核心与 ROS 封装分离）
- 借鉴点：状态定义（x/y/yaw + 速度 + 偏差）、传感器 covariance 调权（打滑=odom 不确定性↑）、只更新可测状态分量（odom 不测 yaw 速度就不融合它）
- 注意：ROS 生态，嵌入式侧只借算法思想，不引入依赖
