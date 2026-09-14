# SensorFusion — 双点横向传感器 + IMU 融合模板

## 定位

**不是独立产品。** 是 Kinematics 运动学库的**宣传素材**和**备用代码模板**。

作为 Kinematics 库的 `examples/dual_sensor_fusion.cpp` 存在，同时本文档解释背后的数学原理和调参方法。

## 为什么降级为模板

三轮分析后的结论：

1. **互补滤波器已有大量现成库**（`tcleg/Six_Axis_Complementary_Filter`、`AIS-Bonn/attitude_estimator` 等），再做一个是第 N+1 个
2. **双点几何航向融合**仅对"前后各有一个横向传感器 + IMU"的特定拓扑有效——竞赛中不到 30% 的队伍用这个配置
3. 代码总量仅 ~50 行，不值得独立封装

但其**教育价值**高——大多数学生不知道"两个横向传感器可以几何计算出航向"。作为文档和示例传播恰好合适。

## 适用场景

- 小车前后各有一个横向误差传感器（灰度阵列、红外对管、电磁电感均可）
- 同时配备 IMU（至少 gyro_z）
- 需要无漂移的航向估计 + 低延迟的角速度响应

## 数学原理

### 几何航向（双传感器差分）

```
前传感器位置 x=+L/2, 读数 e_front
后传感器位置 x=-L/2, 读数 e_back

横向偏移 = (e_front + e_back) / 2
航向误差 = (e_front - e_back) / L        ← 纯几何，不依赖 IMU
```

两个点确定一条直线。知道线在车身坐标系中的位置 → 横向偏移 + 航向误差直接解出。

### IMU 互补滤波

```
融合航向 = α × (上一帧融合航向 + gyro_z × dt)   ← IMU 预测（快但漂）
         + (1-α) × 几何航向                       ← 传感器纠正（准但抖）
```

- `α` 接近 1（如 0.98）：更信任 IMU，响应快但有小漂
- `α` 接近 0（如 0.90）：更信任传感器，更准确但更抖
- 典型值：`α = 0.95~0.98`

### 为什么互补

| 数据源 | 优点 | 缺点 |
|--------|------|------|
| 几何航向 | 无漂移 | 噪声大，传感器抖一下就晃 |
| IMU 积分 | 响应快，平滑 | 积分漂移，长时间偏离 |
| **互补融合** | **无漂移 + 平滑 + 快** | 需要两个传感器 |

## 代码模板（核心 ~30 行）

```cpp
struct DualSensorFusion {
    float alpha;        // 互补滤波系数 (0.95~0.98)
    float baseline;     // 两传感器间距 (m)
    float heading;      // 当前融合航向 (rad)
    float lateral;      // 当前横向偏移 (m)

    void update(float e_front, float e_back, float gyro_z, float dt) {
        // 1. 几何解算
        lateral = (e_front + e_back) * 0.5f;
        float heading_geo = (e_front - e_back) / baseline;

        // 2. 互补滤波
        float heading_gyro = heading + gyro_z * dt;
        heading = alpha * heading_gyro + (1.0f - alpha) * heading_geo;
    }
};
```

完整示例见 `kinematics/examples/dual_sensor_fusion.cpp`。

## 推广价值

作为 Kinematics 的 README 和文档中的 "Use Case" 出现：

> "配上一段 30 行的互补滤波模板，前后两个灰度 + 一个 IMU 就能解算出车的完整姿态——这是电赛/智能车最主流的传感器配置。"

让学生看到：不只是运动学库，还有**配套的传感器融合思路**。降低他们从"有公式"到"写出能跑的车"之间的门槛。

## 文件夹结构

```
sensor_fusion/
├── docs/
│   └── README.md              ← 本文件
└── (实际代码在 kinematics/examples/dual_sensor_fusion.cpp)
```
