# AHRS 姿态解算系统构建指导（草案 v0，待决策）

> 日期：2026-08-22
> 状态：**设计草案，明天拍板是否开工**
> 需求：小车 IMU（先 gyro_z，扩展多维姿态解算）
> 传感器：ICM-20602 = **6 轴（3 陀螺 + 3 加速度计），无磁力计**
> 素材：`legacy/Lib/driver/hal/imu/icm20602/`（驱动）+ 用户 vibecode 过的 Mahony
> 原则：镜像 wheel（最小通用核心 + Config 聚合 + dt 穿透 + 单位约定 + 行为锚点 + 零依赖）

## 1. 为什么需要它

- 小车近期有 IMU gyro_z 需求（转向角/里程计辅助）
- 单用 gyro_z = 纯积分 → **必然漂移**（物理限制，不是代码问题）
- 全姿态（roll/pitch 绝对 + yaw 相对）顺带覆盖 gyro_z 需求，且组件可复用于未来 9 轴

## 2. 原理速查（明天深入时对照）

### 2.1 传感器特性（决定能力边界）

| 传感器 | 测什么 | 特性 | 确定什么 |
|---|---|---|---|
| 陀螺仪 | 角速度 | 短期准，长期漂移 | 相对角度（积分） |
| 加速度计 | 比力（含重力） | 长期准，短期噪声 | roll/pitch 绝对值 |
| 磁力计（无） | 地磁方向 | 长期准 | yaw 绝对值 |

**推论：6 轴 → roll/pitch 绝对（重力参考）+ yaw 纯积分（漂移）。**

### 2.2 融合 = 互补滤波

- 陀螺积分做主干（快，短期准）
- 加速度计做路标修正（慢，长期准）
- 数学家族 = 高通（陀螺）+ 低通（加速度计），与 wheel 的 LPF 同族

### 2.3 Mahony 四步（vibecode 过但没懂的原理）

```
① 姿态预测：陀螺角速度积分更新四元数 q
② 预测重力：按 q 旋转"应指向下的重力"→ 得 ĝ
③ 误差测量：ĝ × 实测加速度 ā 的叉积 = 方向误差 e（叉积自带方向信息）
④ PI 修正： 陀螺角速度 += kp·e + ki·∫e（误差灌回陀螺，抵消漂移）
```

代码对照（vibecode 时看到的魔法数字）：`q0..q3`=四元数；`2*(q1*q3-q0*q2)`=预测重力分量；`ex=(ay*vz-az*vy)`=叉积误差；`gx+=kp*ex`=PI 修正；`q0+=(-q1*gx-q2*gy-q3*gz)*halfT`=四元数积分。

### 2.4 为什么四元数

- 欧拉角 pitch=±90° 万向锁（两轴糊住）+ 三角运算多
- 四元数：无奇点、纯乘加，嵌入式友好

## 3. 能力边界（必须写进文档的承诺）

- 6 轴：yaw 漂移是物理限制 → 测试里标注"预期漂移"
- 要 yaw 绝对参考：加磁力计（9 轴）或外部融合（轮速里程计/视觉）
- 加速度计受振动干扰 → 静止/低速时可信（车体运动时互补增益需权衡）

## 4. 最小通用核心设计（草案，镜像 wheel）

```cpp
// inc/ahrs.hpp（组件边界 = 姿态解算器，驱动/应用分离）
struct AhrsConfig {
    float sample_rate_hz = 1000.0f;   // 参考值（dt 仍穿透实测值）
    float kp = 2.0f;                  // 互补增益（拉回速度）
    float ki = 0.0f;                  // 消静差
    bool  use_mag = false;            // 有磁力计才开（ICM-20602 关）
};

class Ahrs {
public:
    explicit Ahrs(const AhrsConfig &cfg);
    void update(float gx, float gy, float gz,     // 角速度 rad/s
                float ax, float ay, float az,     // 加速度（归一化或 m/s²，单位约定待定）
                float dt);                        // dt 穿透（实测值）
    void reset();
    float roll_deg() const; float pitch_deg() const; float yaw_deg() const;
    // 四元数 getter（导航用）
private:
    AhrsConfig cfg_;
    float q0_=1, q1_=0, q2_=0, q3_=0;
    float integral_fb_[3] = {};       // PI 的 I 项
};
```

与 wheel 同构的原则：Config 聚合 / dt 穿透（实测值非配置值）/ 单位约定（rad/s，文档声明）/ 行为锚点 / 零依赖。

## 5. 测试锚点（测行为不测数值）

| 场景 | 输入 | 期望 |
|---|---|---|
| 静止收敛 | 恒加速度 (0,0,1)，零陀螺 | roll/pitch 从任意初始收敛到 <1°（N 帧内） |
| 积分一致性 | 恒角速度 90°/s，kp=ki=0 | 1s 后 yaw ≈ 90°±容差 |
| 漂移抑制 | 静态 + 陀螺零偏注入 | roll/pitch 不被带跑（yaw 漂移标注预期） |

## 6. 待决策清单（明天拍板）

1. **路线 A/B**：先 gyro_z 单轴（最小）vs 直接 Mahony 6 轴（推荐，额外成本 ~50 行）
2. **组件命名**：`Ahrs` / `AttitudeEstimator` / `ImuFusion`？（三原则评估）
3. **单位约定**：rad/s vs °/s（驱动通常 °/s，解算通常 rad/s——转换放哪层）
4. **加速度输入**：归一化 vs m/s²
5. **项目位置**：`lunokhod/control/imu/` 或独立仓库（镜像 wheel）
6. **9 轴预留**：use_mag 开关现在留还是将来加
7. **与轮速里程计融合**：yaw 漂移是否用编码器辅助（近期需求只到 gyro_z？）

## 7. 参考素材

- legacy 驱动：`legacy/Lib/driver/hal/imu/icm20602/icm.{c,h}`（先读它确认原始量程/单位）
- 用户 vibecode 的 Mahony 代码（对比本指导 2.3 的代码对照表）
- wheel 已沉淀的方法论：`control/wheel/docs/WHEEL_LESSONS.md`
