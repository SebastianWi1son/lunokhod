# Nav Module — Portable Line-Following Navigator

纯算法导航模块，零硬件依赖。适用于差速底盘 + 8 路灰度传感器 + IMU 的巡线小车。

## Dependencies / 依赖

以下头文件必须由目标工程提供（本模块不包含）：

| 文件 | 提供内容 |
|------|---------|
| `dsp.h` | `dsp_ramp_t`, `dsp_ramp_init()`, `dsp_ramp_calc()` |
| `pid.h` | `pid_instance_t`, `pid_init()`, `pid_calculate()`, `pid_reset()` |
| `ff.h` | `ff_rate()`, `ff_bias()`, `ff_gyro_damp()`, `ff_arc_bias()` |
| `<stdint.h>` `<stdbool.h>` | C 标准库 |

**ff.h 注意**: 本模块使用的新 wrapper 签名（`ff_gyro_damp(rate, kd)` 和 `ff_arc_bias(dir, gain)`）去掉了 `base` 参数（PID 与 FF 平行相加）。如果你的 `ff.h` 版本不匹配，参考下方"ff.h 适配"。

## Quick Start / 快速集成

```c
#include "nav.h"

nav_t navigator;    // 全局或静态分配

// 1. 初始化
nav_init(&navigator, kp, kd, tkp, tkd);

// 2. 定义赛道路线
static const nav_action_e route[] = {
    NAV_ACTION_STRAIGHT,
    NAV_ACTION_LEFT,
    NAV_ACTION_RIGHT,
    NAV_ACTION_STOP,
};
nav_route_map_t map = { .nodes = route, .length = sizeof(route)/sizeof(route[0]) };

// 3. 定义调参档案
nav_tune_t tune = {
    .base_speed            = 90.0f,
    .corner_speed_ratio    = 0.85f,
    .max_yaw_rate          = 360.0f,
    .dash_dist_normal      = 1.0f,
    .dash_dist_arc_entry   = 0.5f,
    .dash_dist_arc_exit    = 15.0f,
    .blind_dist_start      = 5.0f,
    .blind_dist_turn       = 0.5f,
    .blind_dist_straight   = 5.0f,
    .gyro_kd               = 0.5f,
    .turn_yaw_tol          = 7.0f,
    .arc_ff_gain           = 13.6f,
    .arc_exit_angle        = 85.0f,
    .arc_enter_yaw_tol     = 15.0f,
    .arc_enter_settle      = 5,
    .blind_route           = NULL,   // 无盲区路线
};

// 4. 启动
nav_start(&navigator, initial_yaw, &map, &tune);

// 5. 每帧调用（100Hz 控制 ISR）
nav_update(&navigator, yaw, yaw_rate, gray, dt, speed, &L_target, &R_target);

// 6. 将 L_target, R_target (RPM) 喂给底盘电机控制
wheel_set_target(&left_wheel, L_target);
wheel_set_target(&right_wheel, R_target);
```

## Architecture / 架构

```
nav.c (dispatch)
 ├─ nav_detect.c    check_node() + commit_action()
 ├─ nav_line.c      统一巡线 (straight / arc)
 ├─ nav_heading.c   统一航向 (turn / pivot / arc_enter)
 ├─ nav_blind.c     盲走序列
 └─ nav_sensor.c    传感器处理
```

状态机：`IDLE → LINE → HEADING → LINE` / `LINE → BLIND → LINE`

## Tuning Guide / 调参指南

### 关键参数

| 参数 | 含义 | 调大效果 | 典型范围 |
|------|------|---------|---------|
| `base_speed` | 直道基础速度 (rpm) | 更快但惯性更大 | 60~120 |
| `gyro_kd` | 直道陀螺阻尼增益 | 蛇形更小，但转向迟钝 | 0.2~1.0 |
| `arc_ff_gain` | 弧线前馈增益 | 弧线跟踪更紧 | 8~20 |
| `dash_dist_normal` | 转弯前冲刺距离 (cm) | 转弯位置更准 | 0.5~3 |
| `blind_dist_turn` | 转弯后盲区 (cm) | 防连续弯误触发 | 0.3~3 |
| `turn_yaw_tol` | 转弯 yaw 关断阈值 (°) | 放宽=快速但不精确 | 5~10 |
| `arc_enter_yaw_tol` | 弧切入 yaw 关断阈值 | 放宽=切入快但不精确 | 10~20 |
| `arc_exit_angle` | 弧线出口角度阈值 (°) | 改变弧线早退/晚退 | 70~95 |

### 连续短弯调参

转弯后 `blind_dist_turn` 设得太长会导致第二个转弯被跳过。先设 0.5cm 测试，逐步增大直到误触发消失。

## Hardware Assumptions / 硬件假设

### 灰度传感器

- 8 个二值传感器，排列为线性阵列
- bit 位置：bit7 = 最左, bit0 = 最右（可通过 `NAV_SENSOR_BIT_LEFT/RIGHT` 配置）
- 黑线宽约 2 个传感器间距

### IMU

- 提供绝对 yaw 角（积分）和瞬时 yaw_rate
- Z 轴正方向 = 逆时针

### 差速底盘

- 两轮独立驱动
- `L = speed + steer, R = speed - steer`（NAV_STEER_POLARITY 可反转）

## Adding Blind Routes / 添加盲走路线

```c
// 断桥过桥: 直行 30cm → 右转 45° → 直行 20cm 寻找线
static const blind_step_t bridge_steps[] = {
    { BLIND_STEP_DASH,   30.0f },
    { BLIND_STEP_ROTATE, 45.0f },
    { BLIND_STEP_DASH,   20.0f },
};

blind_route_t bridge_route = {
    .steps    = bridge_steps,
    .length   = 3,
    .speed    = 70.0f,
    .yaw_rate = 180.0f,
};

tune.blind_route = &bridge_route;

// 在 route map 中触发:
// NAV_ACTION_ENTER_BLIND
```

## Platform Porting Notes / 平台移植注意

### TI C2000 编译器

匿名 union (`ff_config_t`) 需要 C11 支持（TI CGT 20.2.x LTS+）。若使用旧版编译器，改用命名 union：

```c
typedef struct {
    ff_type_e type;
    union {
        struct { float kd; } straight;
        struct { float gain; float direction; } arc;
    } data;                          // ← named
} ff_config_t;
// 访问: cfg.data.straight.kd
```

### Yaw 环绕

当前 `|target_yaw - current_yaw|` 不做 360° 归一化。短赛程（累计 < 2000°）不影响。若需连续长时间导航，在 `heading_exit_ok()` 中添加角度归一化。

### 内存占用

`nav_t` 约 250 字节（含 PID 实例和盲走 buffer），STM32F103 (20KB SRAM) 无压力。

## ff.h 适配

如果 ff.h 的 wrapper 签名与旧式嵌套架构一致（含 `base` 参数），修改为：

```c
// 旧（嵌套架构）
static inline float ff_gyro_damp(float base, float gyro_rate, float kd) {
    return ff_rate(base, gyro_rate, kd);
}
// 新（平行架构）
static inline float ff_gyro_damp(float gyro_rate, float kd) {
    return ff_rate(0.0f, gyro_rate, kd);
}

// 旧
static inline float ff_arc_bias(float base, float direction, float gain) {
    return ff_bias(base, -direction * gain);
}
// 新
static inline float ff_arc_bias(float direction, float gain) {
    return ff_bias(0.0f, -direction * gain);
}
```

基元函数 `ff_rate()` 和 `ff_bias()` 保持不变。

## File Manifest / 文件清单

| 文件 | 行数(约) | 职责 |
|------|---------|------|
| `nav.h` | 250 | API + 类型 + 平台常量 |
| `nav_internal.h` | 60 | 子模块共享声明 |
| `nav.c` | 190 | 状态机 dispatch |
| `nav_detect.c` | 250 | 节点检测 + 动作映射 |
| `nav_line.c` | 110 | 统一巡线 |
| `nav_heading.c` | 100 | 统一航向 |
| `nav_blind.c` | 165 | 盲走序列 |
| `nav_sensor.c` | 135 | 传感器处理 |
