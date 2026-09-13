#include "../nav_internal.h"

/*
 * nav_heading.c — Unified Heading Manoeuvre
 * 统一航向机动 (转弯 / 自转 / 弧切入)
 *
 * All turn / pivot / arc_enter share the same control skeleton:
 *   1. yaw trajectory ramp (smooth slew limiting)
 *   2. turn_pid tracking the profiled yaw
 *   3. differential output (same-direction or opposite-direction)
 *   4. exit when yaw_ok && line_ok (both conditions)
 *
 * Returns true when the manoeuvre is complete; the caller switches state.
 * 返回 true 表示机动完成，调用方负责切换状态。
 */

// ============================================================
// Heading exit check — shared local helper
// 航向退出检测
// ============================================================
static bool heading_exit_ok(nav_t *nav, heading_cfg_t *cfg,
                            uint8_t gray, uint8_t *settle) {
    float yaw_err = nav_abs(nav->target_yaw - nav->current_yaw);
    bool  yaw_ok  = (yaw_err < cfg->exit_yaw_tol);

    /* No sensor mask → yaw-only exit (used by BLIND rotate steps).
       无传感器掩码 → 纯 yaw 退出 (BLIND 旋转步骤使用) */
    if (cfg->exit_sensor_mask == 0) return yaw_ok;

    bool line_ok = (gray & cfg->exit_sensor_mask) != 0;
    if (yaw_ok && line_ok) {
        if (++(*settle) >= cfg->exit_settle_cnt) return true;
    } else {
        *settle = 0;
    }
    return false;
}

// ============================================================
// Unified Heading Update
// 统一航向机动更新
// ============================================================

/**
 * @brief  Per-frame heading manoeuvre update.
 *         每帧航向机动更新。
 * @param  nav   Navigator instance
 * @param  cfg   Heading configuration (wheel mode, limits, exit rules)
 * @param  gray  8-bit gray sensor data
 * @param  dt    Delta time (seconds)
 * @param  L,R   [out] Left/Right wheel target RPM
 * @return true  Manoeuvre complete — caller switches to cfg->exit_state
 *         false Still executing
 */
bool nav_heading_update(nav_t *nav, heading_cfg_t *cfg,
                        uint8_t gray, float dt, float *L, float *R) {
    /* --- 1. yaw trajectory ramp + turn PID --- */
    nav->yaw_ramp.max_rate = nav->tune->max_yaw_rate;
    float prof = dsp_ramp_calc(&nav->yaw_ramp, nav->target_yaw, dt);
    float out  = nav_constrain(
        pid_calculate(&nav->turn_pid, prof, nav->current_yaw, dt),
        -cfg->pid_limit, cfg->pid_limit);

    /* --- 2. differential output --- */
    if (cfg->wheel_mode == WHEEL_SAME) {
        /* Same-direction: rolling turn (TURN, ARC_ENTER) */
        *L = nav_constrain(cfg->speed - out * NAV_STEER_POLARITY, 0.0f, 3600.0f);
        *R = nav_constrain(cfg->speed + out * NAV_STEER_POLARITY, 0.0f, 3600.0f);
    } else {
        /* Opposite-direction: in-place pivot */
        *L = nav_constrain(-out * NAV_STEER_POLARITY, -3600.0f, 3600.0f);
        *R = nav_constrain( out * NAV_STEER_POLARITY, -3600.0f, 3600.0f);
    }

    /* --- 3. exit check --- */
    if (heading_exit_ok(nav, cfg, gray, &nav->heading_settle)) {
        nav->heading_settle = 0;

        /* ARC_ENTER side-effect: carry arc context into subsequent LINE state */
        if (cfg->is_arc_enter) {
            nav->arc_ctx.direction = cfg->arc_direction;
            nav->arc_ctx.start_yaw = nav->current_yaw;
        }

        /* PIVOT side-effect: re-anchor base_yaw after in-place rotation */
        if (cfg->wheel_mode == WHEEL_OPPOSITE) {
            nav->arc_ctx.base_yaw = nav->current_yaw;
        }

        return true;   /* manoeuvre complete */
    }
    return false;
}
