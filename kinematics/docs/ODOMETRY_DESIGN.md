# ODOMETRY_DESIGN.md — lunokhod/kinematics 里程计组件设计

> 状态：**设计草案 v2**（决策点 O1~O9 待拍板）
> 地位：里程计 = 底盘运动系统的**状态侧**核心组件（kinematics 是纯函数瞬时映射，odometry 是有状态积分器 + 数据记录源）

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
- ③ `samples`：原始轮速+指令+位姿+时间戳 → 建模/检测/回放

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
- 轮径在线标定（只留系数修正口）

## 3. 数据流

```
Wheel 实际速度 ×4 → MecanumDrive.forward → Twist（瞬时）→ Odometry.update(twist, tick, dt)
                                                              │
                                              ┌───────────────┼───────────────┐
                                              ▼               ▼               ▼
                                          Pose 位姿      yaw 校准参考    记录样本(可选写入)
                                              │               │               │
                                              ▼               ▼               ▼
                                        导航/上层      → foucault      → 数据导出/回放
```

**关键解耦**：Odometry 吃 **Twist**（瞬时体速度），不吃原始轮速 → 对底盘类型零感知。正解在外部调。

## 4. 数学（积分模型）

```
yaw_new = yaw + wz · dt
x_new   = x + (vx·cos(yaw) − vy·sin(yaw)) · dt      ← 体→世界系旋转（麦轮 vy 必须旋转！）
y_new   = y + (vx·sin(yaw) + vy·cos(yaw)) · dt
```

- 积分法：**欧拉 v1**（梯形法为未来升级项）
- **yaw 存储**：内部**连续累计**（支持多圈）；输出可选 wrap 到 (−π, π]（k2PI 思想）
- 位置 x/y 是世界系；与"forward 输出瞬时 twist"严格区分（那是 kinematics 的活）

## 5. 接口草案（伪代码，拍板后细化）

```cpp
// contracts.hpp 内：Pose 数据契约
struct Pose {
    float x_, y_, yaw_;   // 世界系，m / rad
};

// odometry.hpp 内：记录样本（需求 ③ 的数据形状）
struct OdometrySample {
    uint32_t tick_;       // 时钟刻度（ms/us 计数，零依赖）
    float    ws_[4];      // 原始四轮速（rad/s，打滑检测/建模用）
    Twist    cmd_;        // 指令 twist（期望 vs 实际对照）
    Pose     pose_;       // 当前位姿
    float    dt_;         // 本帧间隔
};

// odometry.hpp 内：记录输出接口（注入式，组件保持纯净）
using SampleSink = void (*)(const OdometrySample&);

// odometry.hpp 内
class Odometry {
public:
    explicit Odometry(float wheel_radius_calib = 1.0f);  // 修正口：轮径标定系数
    void   set_sink(SampleSink sink);                    // 注入记录输出（可选，null = 不记录）
    void   reset();                                      // 位姿清零
    Pose   update(const Twist& body_vel, float dt,
                  uint32_t tick, const float ws[4] = nullptr); // 积分 + 可选记录
    Pose   pose() const;                                 // 只读查询
    float  yaw_ref() const;                              // 校准源出口（给 foucault 的 yaw 参考）
private:
    float  x_, y_, yaw_;
    float  wheel_radius_calib_;
    SampleSink sink_ = nullptr;
};
```

- 单位：m / rad / s（中性化）；状态私有 + 只读查询
- **记录职责**：Odometry 只负责**组装样本 + 触发 sink**，不持有缓冲/文件（消费方自决：PC 建模侧可写 CSV/回放文件，嵌入式侧可接串口导出）

## 6. 决策点清单（拍板用，O1~O4 有依赖一起放）

| # | 决策点 | 选项 | 推荐 |
|---|---|---|---|
| **O1** | 归属 | a) kinematics 库内新文件 `odometry.hpp`（chassis 聚合加入） b) 独立组件目录 c) 独立库 | **a**：依赖 contracts，运动学家族状态侧 |
| **O2** | 输入形态 | a) `update(Twist, dt, tick, ws[])`（底盘无关） b) `update(WheelSpeeds, dt)`（内部调 forward，耦合底盘） | **a**：对底盘零感知 |
| **O3** | Pose 归属 | a) 进 contracts.hpp b) 进 odometry.hpp | **a**：跨组件输出契约归数据层 |
| **O4** | yaw 存储 | a) 内部连续累计 + 输出可选 wrap b) 内部就 wrap | **a**：多圈正确性可测 |
| O5 | 积分法 | 欧拉 vs 梯形 | 欧拉 v1 |
| O6 | 修正口 | v1 只留 wheel_radius_calib | 轮径标定系数起步 |
| O7 | 锚点测试 | 见 §7 | 五锚点 |
| **O8** | 记录形态 | a) 注入式 sink 回调（组件纯净） b) 组件内环形缓冲 c) 独立 Recorder 组件 | **a**：消费方自决，可测性好 |
| **O9** | 记录样本内容 | a) ws×4 + cmd + pose + tick + dt（全量） b) 只 pose + tick | **a**：建模/打滑检测需要原始轮速 |

## 7. 测试锚点（O7）

1. **恒速直行**：vx 恒、wz=0 → x 线性增长，yaw 恒 0
2. **圆轨迹闭合**：vx/wz 恒 → 轨迹半径 ≈ vx/wz（多点采样圆拟合，容差）
3. **yaw 回绕**：wz 大、多圈 → 内部单调累计不跳变；输出 wrap 落 (−π, π] 连续
4. **零速不动**：Twist 全零 → Pose 不变
5. **世界系旋转**：初始 yaw=π/2、纯 vy（麦轮横移）→ 世界系沿 x 移动
6. **记录触发**（O8/O9 拍板后）：注入 sink → 每帧收到样本，字段与输入一致（tick 单调、ws/cmd/pose 正确）
7. **校准参考**：yaw_ref 与内部 yaw 一致（多圈下连续）

## 8. 未来拓展（记账）

- 打滑**检测算法**（样本已含数据：|指令−实际|、轮间一致性、与 IMU 加速度对照）
- 梯形积分升级 / 轮径在线估计 / 里程计协方差（EKF 融合输入）
- 记录导出工具：CSV/回放（对齐 foucault host/replay 模式）
- IMU 辅助位置（foucault 加速度短时修正位置，反向互补）

## 9. 分工与参考（融合在融合层，不在 odometry）

| 组件 | 职责 | 输出 | 消费方 |
|---|---|---|---|
| **Odometry** | 轮速→位姿积分 + 记录 | pose / yaw_ref / samples | 上层 / 融合层 / 建模 |
| **foucault** | IMU→姿态（四元数） | yaw/pitch/roll | 上层 / 融合层 |
| **融合层**（未来组件，可挂 foucault estimator） | 互补/EKF：yaw 互补 + 打滑降权 | 融合位姿 | 导航/定位 |

**融合层实现逻辑（用户闭环落地）**：
- 低频：里程计 yaw_ref 压 IMU 漂移（yaw 校准）
- 高频：IMU 平滑短时抖动
- 打滑：IMU 加速度/角速度与轮速一致性异常 → 里程计协方差调大/降权（不硬丢弃，EKF 调权思想）

**市场参考（已 clone `cyclotron/reference/robot_localization`，36M）**：
- ROS 生态最主流轮式里程计+IMU 融合包（EKF/UKF，15 维状态含 gyro bias，two_d_mode 2D 平面）
- 核心可读：`src/ekf.cpp`、`src/filter_base.cpp`（滤波核心与 ROS 封装分离）
- 借鉴点：状态定义（x/y/yaw + 速度 + 偏差）、传感器 covariance 调权（打滑=odom 不确定性↑）、只更新可测状态分量（odom 不测 yaw 速度就不融合它）
- 注意：ROS 生态，嵌入式侧只借算法思想，不引入依赖
