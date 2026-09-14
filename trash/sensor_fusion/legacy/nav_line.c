#include "nav_internal.h"

/*
 * nav_line.c — Unified Line Tracking (Straight & Arc)
 * 统一巡线 (直道 + 弧线)
 *
 * Control formula / 控制公式:
 *   steer = track_pid(line_error) + ff_term
 *
 *   ff_term depends on ff_config_t.type:
 *     FF_STRAIGHT → ff_gyro_damp(yaw_rate, kd)       gyro rate damping
 *     FF_ARC      → ff_arc_bias(direction, gain)      arc curvature bias
 *
 * Note: gyro damping is NOT applied during arc tracking — the natural
 * yaw rate of an arc is desired behaviour, not oscillation.
 * 注意: 弧线巡迹不叠加陀螺阻尼 — 弧线的自然角速度是期望行为而非振荡。
 *
 * Arc completion: when accumulated yaw turn angle exceeds arc_exit_angle,
 * automatically transitions to BLIND state with a single-step dash route.
 * 弧线完成检测: 累计转角超过 arc_exit_angle 时自动切入 BLIND 盲冲。
 */

/**
 * @brief  Per-frame line tracking update.
 *         每帧巡线更新。
 * @param  nav       Navigator instance
 * @param  cfg       Line configuration (FF type + speed ratio + filter flag)
 * @param  gray      Raw 8-bit gray sensor data
 * @param  yaw_rate  Current angular velocity (°/s, from gyro)
 * @param  dt        Delta time (seconds)
 * @param  L,R       [out] Left/Right wheel target RPM
 */
void nav_line_update(nav_t *nav, line_cfg_t *cfg,
                     uint8_t gray, float yaw_rate, float dt,
                     float *L, float *R) {
    /* --- 1. sensor processing --- */
    uint8_t clean_gray = gray;
    if (cfg->filter_sensor && cfg->ff.type == FF_ARC) {
        clean_gray = nav_filter_line_by_dir(gray, cfg->ff.arc.direction);
    }

    nav->line_error = nav_calc_line_error(nav, clean_gray);
    float track_err = -nav->line_error;

    /* --- 2. track PID with dynamic KP --- */
    float saved_kp = nav->track_pid.kp;
    nav->track_pid.kp = saved_kp * nav_calc_dynamic_kp(track_err);
    float pid_out = pid_calculate(&nav->track_pid, 0.0f, -track_err, dt);
    nav->track_pid.kp = saved_kp;

    /* --- 3. feedforward term (parallel, NOT nested in PID) --- */
    float ff_term = 0.0f;
    switch (cfg->ff.type) {
    case FF_STRAIGHT:
        ff_term = ff_gyro_damp(yaw_rate, cfg->ff.straight.kd);
        break;
    case FF_ARC:
        ff_term = ff_arc_bias(cfg->ff.arc.direction, cfg->ff.arc.gain);
        break;
    }

    /* --- 4. steer output (FF and PID are parallel additions) --- */
    float steer = pid_out + ff_term;

    float speed = nav->base_speed * cfg->speed_ratio;
    *L = nav_constrain(speed + steer * NAV_STEER_POLARITY, 0.0f, 3600.0f);
    *R = nav_constrain(speed - steer * NAV_STEER_POLARITY, 0.0f, 3600.0f);

    /* --- 5. pre_detect_lock maintenance (ported from old tracking) --- */
    if (gray != 0x00 && cfg->ff.type == FF_STRAIGHT) {
        bool on_int = ((gray & 0x80) != 0 || (gray & 0x01) != 0)
                   && ((gray & NAV_GRAY_MASK_MIDDLE) != 0);
        if (!on_int) nav->trigger.pre_detect_lock = 0;
    }

    /* --- 6. arc completion → BLIND transition --- */
    if (cfg->ff.type == FF_ARC && nav->tune != NULL) {
        float turned = nav_abs(nav->current_yaw - nav->arc_ctx.start_yaw);
        if (turned >= nav->tune->arc_exit_angle) {
            /*
             * Arc completed — set exit direction and enter blind dash.
             * 弧线完成 — 设置出口方向并进入盲冲。
             * target_yaw = base_yaw + 180° × direction
             * (point opposite to entry, i.e. the exit tangent direction)
             */
            nav->target_yaw = nav->arc_ctx.base_yaw
                            + 180.0f * nav->arc_ctx.direction;
            nav->arc_ctx.direction = 0.0f;  /* no longer on arc */

            blind_step_t step = { BLIND_STEP_DASH, nav->tune->dash_dist_arc_exit };
            nav_enter_blind(nav, &step, 1,
                            nav->base_speed * nav->tune->corner_speed_ratio,
                            nav->tune->max_yaw_rate);
        }
    }
}
