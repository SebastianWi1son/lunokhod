#include "../nav_internal.h"

/*
 * nav.c — State Machine Dispatch
 * 状态机入口
 *
 * Public API: nav_init / nav_start / nav_stop / nav_update
 *
 * State transitions (handled by this file):
 *   IDLE   → LINE      (nav_start, first node is STRAIGHT or none)
 *          → HEADING   (nav_start, first node is PIVOT / ARC_ENTER)
 *   LINE   → HEADING   (check_node triggers)
 *          → BLIND     (check_node ENTER_BLIND, or arc completion)
 *          → IDLE      (check_node STOP)
 *   HEADING→ LINE      (manoeuvre complete → exit_state)
 *   BLIND  → LINE      (sequence done or sensor recovered)
 */

// ============================================================
// Helper — build line_cfg from current segment context
// 根据当前弧上下文构建巡线配置
// ============================================================
static void build_line_cfg(nav_t *nav) {
    if (nav->arc_ctx.direction != 0.0f) {
        /* Arc mode — curvature bias FF, reduced speed, sensor filtering */
        nav->current_line_cfg = (line_cfg_t){
            .ff = {
                .type = FF_ARC,
                .arc  = { .gain      = nav->tune->arc_ff_gain,
                          .direction = nav->arc_ctx.direction }
            },
            .speed_ratio   = nav->tune->corner_speed_ratio,
            .filter_sensor = true,
        };
    } else {
        /* Straight mode — gyro damping FF, full speed, no sensor filtering */
        nav->current_line_cfg = (line_cfg_t){
            .ff = {
                .type     = FF_STRAIGHT,
                .straight = { .kd = nav->tune->gyro_kd }
            },
            .speed_ratio   = 1.0f,
            .filter_sensor = false,
        };
    }
}

// ============================================================
// nav_init
// ============================================================
void nav_init(nav_t *nav, float kp, float kd, float tkp, float tkd) {
    nav->state                = NAV_STATE_IDLE;
    nav->base_speed           = 0.0f;
    nav->current_yaw          = 0.0f;
    nav->target_yaw           = 0.0f;
    nav->line_error           = 0.0f;
    nav->line_error_last_valid = 0.0f;

    nav->current_map        = NULL;
    nav->tune               = NULL;
    nav->current_node_index = 0;
    nav->pending_action     = NAV_ACTION_NONE;

    /* trigger init */
    nav->trigger.blind_cooldown    = 0.0f;
    nav->trigger.dash_target_dist  = 0.0f;
    nav->trigger.dash_current_dist = 0.0f;
    nav->trigger.pre_detect_lock   = 0;
    nav->trigger.node_debounce     = 0;

    /* arc ctx init */
    nav->arc_ctx.direction  = 0.0f;
    nav->arc_ctx.start_yaw  = 0.0f;
    nav->arc_ctx.base_yaw   = 0.0f;
    nav->arc_ctx.settle_cnt = 0;

    nav->heading_settle     = 0;
    nav->blind_step_index   = 0;
    nav->blind_dist_accum   = 0.0f;

    /* PID init */
    pid_init(&nav->track_pid, kp, 0.0f, kd, 1500.0f, 0.0f, 0.0f, 0.0f, 0.02f);
    pid_init(&nav->turn_pid,  tkp, 0.0f, tkd, 2500.0f, 500.0f, 0.0f, 0.0f, 0.02f);
}

// ============================================================
// nav_start
// ============================================================
void nav_start(nav_t *nav, float yaw, nav_route_map_t *map, nav_tune_t *tune) {
    nav->target_yaw  = yaw;
    nav->current_yaw = yaw;
    nav->arc_ctx.base_yaw = yaw;

    nav->current_map        = map;
    nav->tune               = tune;
    nav->base_speed         = (tune != NULL) ? tune->base_speed : 0.0f;
    nav->current_node_index = 0;
    nav->pending_action     = NAV_ACTION_NONE;

    /* yaw ramp init */
    if (nav->tune) dsp_ramp_init(&nav->yaw_ramp, nav->tune->max_yaw_rate);
    nav->yaw_ramp.prev_out = yaw;

    /* PID reset */
    pid_reset(&nav->track_pid);
    pid_reset(&nav->turn_pid);
    nav->heading_settle = 0;

    /* trigger reset */
    nav->trigger.dash_current_dist = 0.0f;
    nav->trigger.pre_detect_lock   = 0;
    nav->trigger.node_debounce     = 0;

    /* arc context */
    nav->arc_ctx.direction  = 0.0f;
    nav->arc_ctx.start_yaw  = yaw;

    /* --- Handle first node (if it's a PIVOT / ARC_ENTER, start in HEADING) --- */
    if (map != NULL && map->length > 0 && nav->tune != NULL) {
        nav_action_e first = map->nodes[0];
        nav->current_node_index = 1;

        /* Initial heading actions start immediately (no dash for first node).
           起始航向动作立即执行 (首节点无需冲刺距离)。*/
        if (first == NAV_ACTION_PIVOT_LEFT_180
            || first == NAV_ACTION_PIVOT_RIGHT_180
            || first == NAV_ACTION_PIVOT_LEFT
            || first == NAV_ACTION_PIVOT_RIGHT
            || first == NAV_ACTION_ARC_ENTER_LEFT
            || first == NAV_ACTION_ARC_ENTER_RIGHT) {
            nav_commit_action(nav, first);
            nav->arc_ctx.base_yaw = nav->target_yaw;
            return;
        }

        /* Otherwise start in LINE with start blind zone */
        nav->state = NAV_STATE_LINE;
        nav->trigger.blind_cooldown = nav->tune->blind_dist_start;
        build_line_cfg(nav);
    } else {
        nav->state = NAV_STATE_LINE;
        if (nav->tune) nav->trigger.blind_cooldown = nav->tune->blind_dist_start;
        build_line_cfg(nav);
    }
}

// ============================================================
// nav_stop
// ============================================================
void nav_stop(nav_t *nav) {
    nav->state      = NAV_STATE_IDLE;
    nav->base_speed = 0.0f;
    nav->pending_action = NAV_ACTION_NONE;
    nav->trigger.dash_current_dist = 0.0f;
    pid_reset(&nav->track_pid);
    pid_reset(&nav->turn_pid);
}

// ============================================================
// nav_update — per-frame dispatch
// ============================================================
void nav_update(nav_t *nav, float yaw, float yaw_rate,
                uint8_t gray, float dt, float speed, float *L, float *R) {
    if (nav->state == NAV_STATE_IDLE || !nav->tune) {
        *L = 0.0f; *R = 0.0f;
        return;
    }

    nav->current_yaw = yaw;
    float step = nav_abs(speed) * dt;   /* distance travelled this frame (cm) */

    switch (nav->state) {

    /* ---- LINE: sensor-guided line tracking ---- */
    case NAV_STATE_LINE:
        /*
         * check_node() runs first:
         *   - returns true  → node just triggered, skip line action this frame
         *   - returns false → no node, proceed to line tracking
         * pending_action blocks line_update (mid-dash sprint).
         */
        if (!nav_check_node(nav, gray, step)
            && nav->pending_action == NAV_ACTION_NONE) {
            nav_line_update(nav, &nav->current_line_cfg,
                            gray, yaw_rate, dt, L, R);
        }
        break;

    /* ---- HEADING: yaw-guided manoeuvre (turn / pivot / arc_enter) ---- */
    case NAV_STATE_HEADING:
        if (nav_heading_update(nav, &nav->current_heading_cfg,
                               gray, dt, L, R)) {
            /* Manoeuvre complete → transition to exit state */
            nav_state_e next = nav->current_heading_cfg.exit_state;

            if (next == NAV_STATE_LINE) {
                nav->state = NAV_STATE_LINE;
                pid_reset(&nav->track_pid);
                build_line_cfg(nav);          /* arc or straight, based on ctx */
                nav->trigger.blind_cooldown = nav->tune->blind_dist_turn;
            } else {
                nav->state = next;            /* e.g. BLIND (future use) */
            }
        }
        break;

    /* ---- BLIND: multi-step sensor-less sequence ---- */
    case NAV_STATE_BLIND:
        nav_blind_update(nav, gray, step, dt, L, R);
        /* BLIND self-transitions to LINE when done or sensor recovered */
        break;

    default:
        *L = 0.0f; *R = 0.0f;
        break;
    }
}
