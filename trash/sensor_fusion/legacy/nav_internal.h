#ifndef NAV_INTERNAL_H
#define NAV_INTERNAL_H

#include "nav.h"
#include "ff.h"

// ============================================================
// Shared inline utilities — used by all sub-modules
// 共享内联工具函数 — 所有子模块共用 (header-only, 零开销)
// ============================================================

/** Clamp value to [lo, hi]. 限幅。 */
static inline float nav_constrain(float val, float lo, float hi) {
    return (val > hi) ? hi : ((val < lo) ? lo : val);
}

/** Absolute value. 绝对值。 */
static inline float nav_abs(float val) {
    return val > 0.0f ? val : -val;
}

// ============================================================
// nav_sensor.c — sensor processing
// ============================================================
float   nav_calc_line_error(nav_t *nav, uint8_t sensor_val);
float   nav_calc_dynamic_kp(float track_error);
uint8_t nav_filter_line_by_dir(uint8_t gray, float direction);

// ============================================================
// nav_heading.c — heading manoeuvre
// ============================================================
bool nav_heading_update(nav_t *nav, heading_cfg_t *cfg,
                        uint8_t gray, float dt, float *L, float *R);

// ============================================================
// nav_line.c — line tracking
// ============================================================
void nav_line_update(nav_t *nav, line_cfg_t *cfg,
                     uint8_t gray, float yaw_rate, float dt,
                     float *L, float *R);

// ============================================================
// nav_blind.c — blind sequence
// ============================================================
void nav_enter_blind(nav_t *nav, const blind_step_t *steps, uint8_t length,
                     float speed, float yaw_rate);
void nav_blind_update(nav_t *nav, uint8_t gray, float step_dist, float dt,
                      float *L, float *R);

// ============================================================
// nav_detect.c — node detection & action commitment
// ============================================================
bool nav_check_node(nav_t *nav, uint8_t gray, float step_dist);
void nav_commit_action(nav_t *nav, nav_action_e act);

#endif /* NAV_INTERNAL_H */
