#include "../nav_internal.h"

/*
 * nav_detect.c — Node Detection & Action Commitment
 * 节点检测与动作提交
 *
 * Two public functions:
 *   nav_check_node()    — called every frame during LINE state
 *   nav_commit_action() — shared by check_node and nav_start (DRY)
 *
 * Internal mechanism (per check_node invokation):
 *   Phase A: blind_cooldown gate — ignore nodes for N cm after a crossing
 *   Phase B: pending_action dash — sprint forward before committing the turn
 *   Phase C: node detection — intersection pattern + debounce + lock
 */

// ============================================================
// Internal helper — build heading_cfg from nav_action_e + tune
// 内部辅助 — 从 action + tune 构建 heading_cfg
// ============================================================
static void build_heading_cfg(nav_t *nav, nav_action_e act, heading_cfg_t *cfg) {
    float bs = nav->tune->base_speed;
    float cr = nav->tune->corner_speed_ratio;

    switch (act) {
    case NAV_ACTION_LEFT:
    case NAV_ACTION_RIGHT:
        *cfg = (heading_cfg_t){
            .wheel_mode       = WHEEL_SAME,
            .speed            = bs * cr,
            .pid_limit        = 1500.0f,
            .exit_yaw_tol     = nav->tune->turn_yaw_tol,
            .exit_sensor_mask = NAV_GRAY_MASK_MIDDLE,
            .exit_settle_cnt  = 2,
            .exit_state       = NAV_STATE_LINE,
            .is_arc_enter     = false,
            .arc_direction    = 0.0f,
        };
        break;

    case NAV_ACTION_PIVOT_LEFT:
    case NAV_ACTION_PIVOT_RIGHT:
        *cfg = (heading_cfg_t){
            .wheel_mode       = WHEEL_OPPOSITE,
            .speed            = 0.0f,
            .pid_limit        = 2500.0f,
            .exit_yaw_tol     = nav->tune->turn_yaw_tol,
            .exit_sensor_mask = NAV_GRAY_MASK_MIDDLE,
            .exit_settle_cnt  = 2,
            .exit_state       = NAV_STATE_LINE,
            .is_arc_enter     = false,
            .arc_direction    = 0.0f,
        };
        break;

    case NAV_ACTION_PIVOT_LEFT_180:
    case NAV_ACTION_PIVOT_RIGHT_180:
        *cfg = (heading_cfg_t){
            .wheel_mode       = WHEEL_OPPOSITE,
            .speed            = 0.0f,
            .pid_limit        = 2500.0f,
            .exit_yaw_tol     = nav->tune->turn_yaw_tol,
            .exit_sensor_mask = NAV_GRAY_MASK_MIDDLE,
            .exit_settle_cnt  = 2,
            .exit_state       = NAV_STATE_LINE,
            .is_arc_enter     = false,
            .arc_direction    = 0.0f,
        };
        break;

    case NAV_ACTION_ARC_ENTER_LEFT:
        *cfg = (heading_cfg_t){
            .wheel_mode       = WHEEL_SAME,
            .speed            = bs * cr * 0.5f,
            .pid_limit        = 250.0f,
            .exit_yaw_tol     = nav->tune->arc_enter_yaw_tol,
            .exit_sensor_mask = NAV_GRAY_MASK_INNER_R,  /* right side inner (bits 0-3) */
            .exit_settle_cnt  = nav->tune->arc_enter_settle,
            .exit_state       = NAV_STATE_LINE,
            .is_arc_enter     = true,
            .arc_direction    = 1.0f,   /* left arc */
        };
        break;

    case NAV_ACTION_ARC_ENTER_RIGHT:
        *cfg = (heading_cfg_t){
            .wheel_mode       = WHEEL_SAME,
            .speed            = bs * cr * 0.5f,
            .pid_limit        = 250.0f,
            .exit_yaw_tol     = nav->tune->arc_enter_yaw_tol,
            .exit_sensor_mask = NAV_GRAY_MASK_INNER_L,  /* left side inner (bits 4-7) */
            .exit_settle_cnt  = nav->tune->arc_enter_settle,
            .exit_state       = NAV_STATE_LINE,
            .is_arc_enter     = true,
            .arc_direction    = -1.0f,  /* right arc */
        };
        break;

    default:
        break;  /* STRAIGHT / STOP / ENTER_BLIND handled by caller */
    }
}

// ============================================================
// nav_commit_action — shared mapping (DRY)
// 动作提交 — nav_start 与 nav_check_node 共用
// ============================================================

/**
 * @brief  Execute a route action: change target_yaw, switch state, setup config.
 *         执行路由动作：改目标航向、切状态、配置 heading/blind cfg。
 *
 *         Called by nav_check_node (when dash completes) and nav_start (first node).
 *         由 nav_check_node (冲刺完成时) 和 nav_start (首节点) 共用。
 */
void nav_commit_action(nav_t *nav, nav_action_e act) {
    switch (act) {
    case NAV_ACTION_STRAIGHT:
        nav->trigger.blind_cooldown = nav->tune->blind_dist_straight;
        return;  /* no state change */

    case NAV_ACTION_STOP:
        nav->state      = NAV_STATE_IDLE;
        nav->base_speed = 0.0f;
        return;

    case NAV_ACTION_LEFT:
        nav->target_yaw += 90.0f;
        nav->state       = NAV_STATE_HEADING;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_RIGHT:
        nav->target_yaw -= 90.0f;
        nav->state       = NAV_STATE_HEADING;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_PIVOT_LEFT:
        nav->target_yaw += 90.0f;
        nav->state       = NAV_STATE_HEADING;
        nav->arc_ctx.base_yaw = nav->target_yaw;  /* pre-anchor for arc later */
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_PIVOT_RIGHT:
        nav->target_yaw -= 90.0f;
        nav->state       = NAV_STATE_HEADING;
        nav->arc_ctx.base_yaw = nav->target_yaw;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_PIVOT_LEFT_180:
        nav->target_yaw += 180.0f;
        nav->state       = NAV_STATE_HEADING;
        nav->arc_ctx.base_yaw = nav->target_yaw;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_PIVOT_RIGHT_180:
        nav->target_yaw -= 180.0f;
        nav->state       = NAV_STATE_HEADING;
        nav->arc_ctx.base_yaw = nav->target_yaw;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_ARC_ENTER_LEFT:
        nav->target_yaw             = nav->arc_ctx.base_yaw + 120.0f;
        nav->arc_ctx.direction      = 1.0f;
        nav->arc_ctx.start_yaw      = nav->current_yaw;
        nav->arc_ctx.settle_cnt     = 0;
        nav->state                  = NAV_STATE_HEADING;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_ARC_ENTER_RIGHT:
        nav->target_yaw             = nav->arc_ctx.base_yaw - 120.0f;
        nav->arc_ctx.direction      = -1.0f;
        nav->arc_ctx.start_yaw      = nav->current_yaw;
        nav->arc_ctx.settle_cnt     = 0;
        nav->state                  = NAV_STATE_HEADING;
        build_heading_cfg(nav, act, &nav->current_heading_cfg);
        break;

    case NAV_ACTION_ENTER_BLIND:
        if (nav->tune->blind_route != NULL) {
            blind_route_t *br = nav->tune->blind_route;
            nav_enter_blind(nav, br->steps, br->length, br->speed, br->yaw_rate);
        }
        return;

    default:
        return;
    }

    /* Heading entry housekeeping */
    nav->yaw_ramp.prev_out = nav->current_yaw;
    pid_reset(&nav->turn_pid);
    nav->heading_settle = 0;
}

// ============================================================
// nav_check_node — per-frame node detection during LINE
// 节点检测 — LINE 状态每帧调用
// ============================================================

/**
 * @brief  Detect and consume route-map nodes.
 *         检测并消费路线地图节点。
 *
 *         Only meaningful during LINE state (guarded by caller).
 *         仅在 LINE 状态有效 (调用方保证)。
 *
 * @return true   Node triggered and committed (caller should skip line action this frame).
 *         false  No node triggered, or mid-dash.
 */
bool nav_check_node(nav_t *nav, uint8_t gray, float step_dist) {
    /* --- guards --- */
    if (!nav->current_map || !nav->tune
        || nav->current_node_index >= nav->current_map->length)
        return false;

    /* ================================================================
       Phase A: Blind cooldown — distance-based node suppression
       阶段 A: 盲区冷却 — 按距离抑制误触发
       ================================================================ */
    if (nav->trigger.blind_cooldown > 0.0f
        && nav->pending_action == NAV_ACTION_NONE) {
        nav->trigger.blind_cooldown -= step_dist;
        return false;
    }

    /* ================================================================
       Phase B: Pending action dash — sprint before committing turn
       阶段 B: 冲刺管理 — 在提交转弯前冲刺到合适位置
       ================================================================ */
    if (nav->pending_action != NAV_ACTION_NONE) {
        nav->trigger.dash_current_dist += step_dist;
        if (nav->trigger.dash_current_dist >= nav->trigger.dash_target_dist) {
            nav_action_e act = nav->pending_action;
            nav->pending_action = NAV_ACTION_NONE;
            nav_commit_action(nav, act);
            return true;  /* state changed — caller skips line action */
        }
        return false;  /* still dashing */
    }

    /* ================================================================
       Phase C: Node detection — intersection pattern + debounce
       阶段 C: 节点检测 — 路口图样 + 去抖
       ================================================================ */
    if (gray == 0x00) return false;

    /* Intersection detection: edge sensors + center fill.
       路口检测: 边缘传感器 + 中线填充。*/
    bool edge   = ((gray & (1 << NAV_SENSOR_BIT_LEFT))  != 0)
               || ((gray & (1 << NAV_SENSOR_BIT_RIGHT)) != 0);

    /* Count left-half and right-half dark sensors for crossroad detection */
    uint8_t lc = 0, rc = 0;
    for (int i = 0; i < NAV_SENSOR_COUNT / 2; i++) {
        if (gray & (1 << (NAV_SENSOR_BIT_RIGHT + i))) rc++;
    }
    for (int i = NAV_SENSOR_COUNT / 2; i < NAV_SENSOR_COUNT; i++) {
        if (gray & (1 << (NAV_SENSOR_BIT_RIGHT + i))) lc++;
    }
    bool on_int = (lc >= 4) || (rc >= 4);

    /* Pre-detect lock: prevent re-trigger on the same intersection.
       预检测锁: 必须先离开路口状态再进入，才计为新的路口。*/
    if (nav->trigger.pre_detect_lock) {
        if (on_int) return false;
        nav->trigger.pre_detect_lock = 0;
    }

    /* Debounce: require N consecutive frames of intersection to confirm.
       去抖: 连续 N 帧确认。*/
    if (on_int) {
        if (++nav->trigger.node_debounce < 2) return false;
    } else {
        nav->trigger.node_debounce = 0;
        return false;
    }

    /* ================================================================
       Node confirmed — consume route and schedule action
       节点确认 — 消费路由并调度动作
       ================================================================ */
    nav->trigger.pre_detect_lock = 1;
    nav->trigger.node_debounce   = 0;

    nav_action_e act = nav->current_map->nodes[nav->current_node_index++];

    switch (act) {
    case NAV_ACTION_STRAIGHT:
        /* Set blind cooldown and continue tracking — no dash needed */
        nav->trigger.blind_cooldown = nav->tune->blind_dist_straight;
        return false;

    case NAV_ACTION_STOP:
        nav->state      = NAV_STATE_IDLE;
        nav->base_speed = 0.0f;
        return true;

    case NAV_ACTION_ENTER_BLIND:
        if (nav->tune->blind_route != NULL) {
            blind_route_t *br = nav->tune->blind_route;
            nav_enter_blind(nav, br->steps, br->length, br->speed, br->yaw_rate);
        }
        return true;

    default:
        /* All heading actions: set pending_action, begin dash sprint.
           所有航向动作: 设置 pending_action, 开始冲刺。*/
        nav->pending_action = act;
        nav->trigger.dash_current_dist = 0.0f;

        if (act == NAV_ACTION_ARC_ENTER_LEFT
            || act == NAV_ACTION_ARC_ENTER_RIGHT) {
            nav->trigger.dash_target_dist = nav->tune->dash_dist_arc_entry;
        } else {
            nav->trigger.dash_target_dist = nav->tune->dash_dist_normal;
        }

        /* Extended blind cooldown: prevent node re-detection during dash */
        nav->trigger.blind_cooldown = nav->trigger.dash_target_dist + 1.5f;
        return false;
    }
}
