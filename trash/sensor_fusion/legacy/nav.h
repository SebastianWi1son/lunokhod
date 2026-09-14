#ifndef NAV_H
#define NAV_H

#include "pid.h"
#include "dsp.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================
// Platform Convention Constants — change per hardware
// 平台约定常量 — 更换硬件/传感器布局时修改此处
// ============================================================
#define NAV_SENSOR_COUNT       8       // 灰度传感器数量 (0=disable sensor filtering)
#define NAV_SENSOR_BIT_LEFT    7       // 最左侧传感器对应 bit 位置 (0~7)
#define NAV_SENSOR_BIT_RIGHT   0       // 最右侧传感器对应 bit 位置
#define NAV_GRAY_MASK_MIDDLE   0x3C    // 中间 4bit 掩码 (bits 2-5)
#define NAV_GRAY_MASK_INNER_L  0xF0    // 左侧内侧 4bit (bits 4-7, 右弧 inner)
#define NAV_GRAY_MASK_INNER_R  0x0F    // 右侧内侧 4bit (bits 0-3, 左弧 inner)
#define NAV_STEER_POLARITY     1.0f    // steer 极性: +1=正 steer 左转, -1=反转

// ============================================================
// Route Actions — lexical, used in route maps
// 路由动作枚举 — 语义化，面向路线定义
// ============================================================
typedef enum {
    NAV_ACTION_NONE = 0,
    NAV_ACTION_STRAIGHT,            // 直行通过路口
    NAV_ACTION_STOP,                // 终点停车
    NAV_ACTION_LEFT,                // 行进左转 90°
    NAV_ACTION_RIGHT,               // 行进右转 90°
    NAV_ACTION_PIVOT_LEFT,          // 原地左转 90°
    NAV_ACTION_PIVOT_RIGHT,         // 原地右转 90°
    NAV_ACTION_PIVOT_LEFT_180,      // 原地左转 180°
    NAV_ACTION_PIVOT_RIGHT_180,     // 原地右转 180°
    NAV_ACTION_ARC_ENTER_LEFT,      // 行进切入左弧
    NAV_ACTION_ARC_ENTER_RIGHT,     // 行进切入右弧
    NAV_ACTION_ENTER_BLIND,         // 进入盲走序列 (从 tune->blind_route 读取路线)
} nav_action_e;

typedef struct {
    const nav_action_e *nodes;      // 动作数组 (存于 ROM)
    uint8_t              length;    // 节点数量
} nav_route_map_t;

// ============================================================
// State Machine
// 状态机
// ============================================================
typedef enum {
    NAV_STATE_IDLE,                 // 停机
    NAV_STATE_LINE,                 // 线引导巡线 (直道/弧线统一)
    NAV_STATE_HEADING,              // 航向机动 (转弯/自转/弧切入统一)
    NAV_STATE_BLIND,                // 盲走序列 (多段 DASH+ROTATE)
} nav_state_e;

// ============================================================
// FF Configuration — tagged union for straight / arc
// 前馈配置 — tagged union 区分直道与弧线参数
// ============================================================
typedef enum { FF_STRAIGHT, FF_ARC } ff_type_e;

/*
 * C11 anonymous union — supported by arm-none-eabi-gcc and TI CGT 20.2+.
 * 匿名 union 需要 C11 支持，ARM GCC / TI CGT 20.2+ 均可用。
 *
 * Usage / 用法:
 *   ff_config_t cfg = { .type = FF_STRAIGHT, .straight = { .kd = 0.5f } };
 *   ff_config_t cfg = { .type = FF_ARC, .arc = { .gain = 13.6f, .direction = 1.0f } };
 *
 * Switch / 读取:
 *   switch (cfg.type) {
 *   case FF_STRAIGHT: term = ff_gyro_damp(rate, cfg.straight.kd); break;
 *   case FF_ARC:      term = ff_arc_bias(cfg.arc.direction, cfg.arc.gain); break;
 *   }
 *
 * Adding a 3rd FF type / 新增第三种前馈:
 *   1. add entry to ff_type_e
 *   2. add struct to union
 *   3. add case in all switch(cfg.type) blocks
 */
typedef struct {
    ff_type_e type;
    union {
        struct { float kd; }             straight;   // FF_STRAIGHT
        struct { float gain; float direction; } arc; // FF_ARC
    };
} ff_config_t;

// ============================================================
// Line Tracking Configuration
// 巡线配置
// ============================================================
typedef struct {
    ff_config_t ff;             // 前馈配置 (type + params)
    float       speed_ratio;    // 速度比例: 直道 1.0, 弧线 0.85
    bool        filter_sensor;  // 弧线: true (方向性传感器过滤)
} line_cfg_t;

// ============================================================
// Heading Manoeuvre Configuration
// 航向机动配置
// ============================================================
typedef enum { WHEEL_SAME, WHEEL_OPPOSITE } wheel_mode_e;

typedef struct {
    wheel_mode_e wheel_mode;        // 同向=行进弯, 反向=原地转
    float        speed;             // 行进速度 (自转: 0)
    float        pid_limit;         // PID 输出限幅 (绝对值)
    float        exit_yaw_tol;      // yaw 误差关断阈值 (°)
    uint8_t      exit_sensor_mask;  // 退出需看到的传感器位掩码
    uint8_t      exit_settle_cnt;   // 稳定持续帧数
    nav_state_e  exit_state;        // 退出后目标状态
    bool         is_arc_enter;      // true: 退出时设置 arc_ctx (供 LINE 使用)
    float        arc_direction;     // 弧方向 (±1.0), 仅 is_arc_enter 有效
} heading_cfg_t;

// ============================================================
// Blind Route — multi-step sequence for wireless zones
// 盲走路线的单步定义
// ============================================================
#define NAV_BLIND_MAX_STEPS 4   // 盲走序列最大步骤数

typedef enum { BLIND_STEP_DASH, BLIND_STEP_ROTATE } blind_step_type_e;

typedef struct {
    blind_step_type_e type;
    float             value;     // DASH: 距离 (cm), ROTATE: 相对转角 (°)
} blind_step_t;

typedef struct {
    const blind_step_t *steps;  // 步骤数组 (存于 ROM)
    uint8_t             length; // 步骤数量
    float               speed;  // 盲走基础速度
    float               yaw_rate; // 旋转角速度 (°/s)
} blind_route_t;

// ============================================================
// Unified Tune Profile — single source of all tunable params
// 统一调参档案
// ============================================================
typedef struct {
    // ---- 基础 ----
    float base_speed;              // 巡线基础速度 (rpm)

    // ---- 冲刺距离 ----
    float dash_dist_normal;        // 直道转弯前冲刺 (cm)
    float dash_dist_pre_turn;      // 连续弯预检测冲刺 (cm)
    float dash_dist_arc_entry;     // 弧线入口冲刺 (cm)
    float dash_dist_arc_exit;      // 弧线出口盲冲 (cm)

    // ---- 盲区距离 ----
    float blind_dist_start;        // 起步盲区 (cm)
    float blind_dist_turn;         // 转弯后盲区 (cm)
    float blind_dist_straight;     // 直行过路口后盲区 (cm)

    // ---- 降速 ----
    float corner_speed_ratio;      // 转弯/弧线速度比例
    float max_yaw_rate;            // yaw 斜坡限制最大角速度 (°/s)

    // ---- 陀螺阻尼 ----
    float gyro_kd;                 // 直道陀螺角速度阻尼增益

    // ---- 弧线 ----
    float arc_ff_gain;             // 弧线前馈增益
    float arc_exit_angle;          // 弧线出口 IMU 角度阈值 (°)

    // ---- 航向退出容差 ----
    float   turn_yaw_tol;          // 转弯 yaw 关断阈值 (°, e.g. 7)
    float   arc_enter_yaw_tol;     // 弧切入 yaw 关断阈值 (°, e.g. 15)
    uint8_t arc_enter_settle;      // 弧切入稳定帧数 (e.g. 5)

    // ---- BLIND (预留) ----
    blind_route_t *blind_route;    // ENTER_BLIND 时使用的盲走路线 (NULL=未配置)
} nav_tune_t;

// ============================================================
// Navigator Control Block
// 导航主控块
// ============================================================
typedef struct {
    // ---- runtime state ----
    nav_state_e   state;
    float         base_speed;
    float         current_yaw;
    float         target_yaw;
    float         line_error;
    float         line_error_last_valid;   // per-instance hold (reentrant)

    // ---- route ----
    nav_route_map_t *current_map;
    nav_tune_t      *tune;
    uint8_t          current_node_index;
    nav_action_e     pending_action;

    // ---- per-mode configs (set at entry) ----
    line_cfg_t    current_line_cfg;
    heading_cfg_t current_heading_cfg;
    blind_route_t *current_blind_route;     // ptr to tune or runtime buffer
    uint8_t        blind_step_index;
    float          blind_dist_accum;
    uint8_t        heading_settle;          // heading exit debounce counter
    blind_step_t   blind_step_buf[NAV_BLIND_MAX_STEPS]; // runtime step buffer
    blind_route_t  blind_route_buf;         // runtime route buffer

    // ---- algorithmic blocks ----
    dsp_ramp_t      yaw_ramp;
    pid_instance_t  track_pid;
    pid_instance_t  turn_pid;

    // ---- internal trigger state ----
    struct {
        float   blind_cooldown;     // 盲区冷却距离 (cm)
        float   dash_target_dist;   // 冲刺目标距离 (cm)
        float   dash_current_dist;  // 冲刺已走距离 (cm)
        uint8_t pre_detect_lock;    // 预检测锁 (防同路口重复触发)
        uint8_t node_debounce;      // 节点去抖计数
    } trigger;

    // ---- arc context (carried across states) ----
    struct {
        float   direction;          // 弧方向 (±1.0)
        float   start_yaw;          // 弧开始时的 yaw
        float   base_yaw;           // 弧基准 yaw (pivot 后重锚)
        uint8_t settle_cnt;         // 稳定计数
    } arc_ctx;
} nav_t;

// ============================================================
// Public API
// ============================================================

/**
 * @brief  Initialise navigator instance.
 *         初始化导航器实例。
 * @param  nav  Pointer to nav_t instance
 * @param  kp   Tracking PID proportional gain
 * @param  kd   Tracking PID derivative gain
 * @param  tkp  Turn PID proportional gain
 * @param  tkd  Turn PID derivative gain
 */
void nav_init(nav_t *nav, float kp, float kd, float tkp, float tkd);

/**
 * @brief  Start a mission.
 *         启动任务。
 * @param  nav  Navigator instance
 * @param  yaw  Initial absolute yaw angle (from IMU)
 * @param  map  Route map for this mission
 * @param  tune Tune profile for this mission
 */
void nav_start(nav_t *nav, float yaw, nav_route_map_t *map, nav_tune_t *tune);

/**
 * @brief  Stop mission immediately.
 *         立即停止任务。
 */
void nav_stop(nav_t *nav);

/**
 * @brief  Per-frame update — call from 100 Hz (or configurable) control ISR.
 *         每帧更新 — 由控制 ISR 调用。
 * @param  nav       Navigator instance
 * @param  yaw       Current absolute yaw angle (°, from IMU)
 * @param  yaw_rate  Current angular velocity (°/s, from gyro)
 * @param  gray      8-bit gray sensor data
 * @param  dt        Delta time (seconds) since last call
 * @param  speed     Current real ground speed (cm/s, from encoder)
 * @param  L         [out] Left wheel target RPM
 * @param  R         [out] Right wheel target RPM
 */
void nav_update(nav_t *nav, float yaw, float yaw_rate,
                uint8_t gray, float dt, float speed, float *L, float *R);

#endif /* NAV_H */
