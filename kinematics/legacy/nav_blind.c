#include "../nav_internal.h"

/*
 * nav_blind.c — Blind Manoeuvre Sequence (multi-step)
 * 盲走序列 (多段机动)
 *
 * Internal mini state machine:
 *   BLIND_DASH    → drive straight for value cm, check sensor each frame
 *   BLIND_ROTATE  → rotate by value degrees, no sensor exit
 *
 * Each BLIND_STEP_DASH checks the gray sensor and can exit early to LINE
 * if valid line data reappears, improving recovery speed.
 * 每个 DASH 步骤每帧检查灰度传感器，若重新检测到有效线则提前退出到 LINE。
 */

/* ================================================================ */
/*  nav_enter_blind — set up BLIND state from any caller            */
/*  进入盲走 — 供 nav_line (弧完成) 和 nav_detect (ENTER_BLIND) 调用 */
/* ================================================================ */

/**
 * @brief  Enter BLIND state with a step sequence.
 *         进入盲走状态。
 *
 *         Copies step data into nav_t's internal buffer — caller may use
 *         stack-allocated data safely.
 *         将步骤数据复制到 nav_t 内部 buffer — 调用方可安全使用栈上临时数据。
 *
 * @param  nav      Navigator instance
 * @param  steps    Step array (copied into nav->blind_step_buf)
 * @param  length   Number of steps (≤ NAV_BLIND_MAX_STEPS)
 * @param  speed    Base speed for blind movement
 * @param  yaw_rate Yaw rate for rotation steps
 */
void nav_enter_blind(nav_t *nav, const blind_step_t *steps, uint8_t length,
                     float speed, float yaw_rate) {
    if (length > NAV_BLIND_MAX_STEPS) length = NAV_BLIND_MAX_STEPS;

    /* Copy steps into nav-owned buffer */
    for (uint8_t i = 0; i < length; i++) {
        nav->blind_step_buf[i] = steps[i];
    }

    /* Build runtime route pointing to the buffer */
    nav->blind_route_buf = (blind_route_t){
        .steps    = nav->blind_step_buf,
        .length   = length,
        .speed    = speed,
        .yaw_rate = yaw_rate,
    };

    nav->current_blind_route = &nav->blind_route_buf;
    nav->state               = NAV_STATE_BLIND;
    nav->blind_step_index    = 0;
    nav->blind_dist_accum    = 0.0f;
    nav->yaw_ramp.prev_out   = nav->current_yaw;
    pid_reset(&nav->turn_pid);
}

/* ================================================================ */
/*  nav_blind_update — per-frame blind sequence step                */
/*  盲走序列每帧更新                                                  */
/* ================================================================ */

/**
 * @brief  Per-frame blind sequence update.
 *         每帧盲走序列更新。
 * @param  nav       Navigator instance
 * @param  gray      8-bit gray sensor data (for early exit during DASH)
 * @param  step_dist Distance travelled this frame (cm, |speed| * dt)
 * @param  dt        Delta time (seconds)
 * @param  L,R       [out] Left/Right wheel target RPM
 */
void nav_blind_update(nav_t *nav, uint8_t gray, float step_dist, float dt,
                      float *L, float *R) {
    blind_route_t *route = nav->current_blind_route;

    if (route == NULL || route->length == 0) {
        /* No route — fallback to LINE with blind cooldown */
        nav->state = NAV_STATE_LINE;
        pid_reset(&nav->track_pid);
        if (nav->tune) nav->trigger.blind_cooldown = nav->tune->blind_dist_straight;
        *L = *R = 0.0f;
        return;
    }

    blind_step_t step = route->steps[nav->blind_step_index];

    switch (step.type) {

    /* ---- DASH: drive straight for distance ---- */
    case BLIND_STEP_DASH: {
        /* Distance accounting */
        nav->blind_dist_accum += step_dist;

        /* Heading hold: lock current target_yaw, dampen drift */
        nav->yaw_ramp.max_rate = route->yaw_rate;
        float prof = dsp_ramp_calc(&nav->yaw_ramp, nav->target_yaw, dt);
        float out  = nav_constrain(
            pid_calculate(&nav->turn_pid, prof, nav->current_yaw, dt),
            -250.0f, 250.0f); /* small authority — just heading hold */

        float s = route->speed;
        *L = nav_constrain(s - out * NAV_STEER_POLARITY, 0.0f, 3600.0f);
        *R = nav_constrain(s + out * NAV_STEER_POLARITY, 0.0f, 3600.0f);

        /* Early exit: sensor recovered line → go back to LINE */
        if (gray != 0x00) {
            nav->state = NAV_STATE_LINE;
            pid_reset(&nav->track_pid);
            if (nav->tune) nav->trigger.blind_cooldown = nav->tune->blind_dist_turn;
            return;
        }

        /* Distance reached → next step */
        if (nav->blind_dist_accum >= step.value) {
            nav->blind_dist_accum = 0.0f;
            nav->blind_step_index++;
        }
        break;
    }

    /* ---- ROTATE: change heading by relative angle ---- */
    case BLIND_STEP_ROTATE: {
        /* Set target yaw then delegate to heading control */
        nav->target_yaw += step.value;

        /* Build a minimal heading_cfg for this rotation step */
        heading_cfg_t rot_cfg = {
            .wheel_mode       = WHEEL_SAME,   /* rolling rotation */
            .speed            = route->speed,
            .pid_limit        = 1500.0f,
            .exit_yaw_tol     = 7.0f,         /* tight tolerance */
            .exit_sensor_mask = 0,            /* yaw-only exit (no sensor) */
            .exit_settle_cnt  = 1,
            .exit_state       = NAV_STATE_BLIND,
            .is_arc_enter     = false,
            .arc_direction    = 0.0f,
        };

        /* Delegate to heading control; when done, advance step */
        if (nav_heading_update(nav, &rot_cfg, gray, dt, L, R)) {
            nav->blind_step_index++;
        }
        break;
    }

    } /* switch */

    /* ---- All steps exhausted → search for line ---- */
    if (nav->blind_step_index >= route->length) {
        if (gray != 0x00) {
            nav->state = NAV_STATE_LINE;
            pid_reset(&nav->track_pid);
            if (nav->tune) nav->trigger.blind_cooldown = nav->tune->blind_dist_straight;
        } else {
            /* Continue straight at route speed */
            nav->yaw_ramp.max_rate = route->yaw_rate;
            float prof = dsp_ramp_calc(&nav->yaw_ramp, nav->target_yaw, dt);
            float out  = nav_constrain(
                pid_calculate(&nav->turn_pid, prof, nav->current_yaw, dt),
                -250.0f, 250.0f);
            float s = route->speed;
            *L = nav_constrain(s - out * NAV_STEER_POLARITY, 0.0f, 3600.0f);
            *R = nav_constrain(s + out * NAV_STEER_POLARITY, 0.0f, 3600.0f);
        }
    }
}
