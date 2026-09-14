#include "nav_internal.h"

// ============================================================
// Line Error Lookup — 8-bit gray pattern → float error
// 巡线误差查表 — 8bit 灰度二值图样 → 浮点误差值
// ============================================================

/**
 * @brief  Map 8-bit gray sensor pattern to line error (-7..+7).
 *         Holds last valid value when sensor loses the line (0x00 / unmatched).
 *         8bit 灰度 → 误差值。丢线时保持上次有效值。
 *
 *         Error sign convention (check NAV_STEER_POLARITY):
 *           正误差 = 线偏右 (sensor bit convention: 0=right, 7=left)
 *           负误差 = 线偏左
 *
 * @param  nav        Navigator instance (holds last_valid)
 * @param  sensor_val 8-bit gray sensor reading
 * @return Line error (-7.0 .. +7.0)
 */
float nav_calc_line_error(nav_t *nav, uint8_t sensor_val) {
    float err;
    switch (sensor_val) {
    case 0b00011000: err =  0.0f; break;   // 线居中
    case 0b00001000: err =  1.0f; break;   // 偏右 1
    case 0b00001100: err =  2.0f; break;   // 偏右 2
    case 0b00000100: err =  3.0f; break;
    case 0b00000110: err =  4.0f; break;
    case 0b00000010: err =  5.0f; break;
    case 0b00000011: err =  6.0f; break;
    case 0b00000001: err =  7.0f; break;
    case 0b00010000: err = -1.0f; break;   // 偏左 1
    case 0b00110000: err = -2.0f; break;   // 偏左 2
    case 0b00100000: err = -3.0f; break;
    case 0b01100000: err = -4.0f; break;
    case 0b01000000: err = -5.0f; break;
    case 0b11000000: err = -6.0f; break;
    case 0b10000000: err = -7.0f; break;
    case 0b11111111: err =  0.0f; break;   // 全黑=十字中心, 视为居中
    case 0b00000000:
    default:         err = nav->line_error_last_valid; break; // 丢线保持
    }
    nav->line_error_last_valid = err;
    return err;
}

/**
 * @brief  Dynamic KP multiplier — amplifies correction when error is large.
 *         动态 KP 倍率 — 大误差时增益放大，三次函数平滑过渡。
 *
 *         Max multiplier ≈ 2.2 at error=7.
 *
 * @param  track_error Absolute line error
 * @return KP multiplier (1.0 .. ~2.2)
 */
float nav_calc_dynamic_kp(float track_error) {
    float ae   = track_error > 0.0f ? track_error : -track_error;
    float base = (ae >= 1.0f && ae <= 3.0f) ? 0.2f : 0.0f;
    float r    = nav_constrain(ae / 7.0f, 0.0f, 1.0f);
    return 1.0f + base + 1.0f * r * r * r;
}

// ============================================================
// Directional Sensor Filter — arc line extraction
// 方向性传感器过滤 — 弧线寻迹时提取内侧真实线
// ============================================================

/**
 * @brief  Extract the real line from the correct side during arc tracking,
 *         discarding ghost/mislead lines on the opposite side.
 *         弧线寻迹时从正确方向提取真实线，丢弃另一侧的误导线。
 *
 *         LEFT  arc (direction > 0): line appears on right side (bits low),
 *                scan from bit0 upward, keep first contiguous dark group.
 *         RIGHT arc (direction < 0): line appears on left side (bits high),
 *                scan from bit7 downward, keep first contiguous dark group.
 *
 *         The result is fed directly into nav_calc_line_error().
 *         输出可直接传入 nav_calc_line_error()。
 *
 * @param  gray       Raw 8-bit gray sensor data
 * @param  direction  Arc direction: >0 = left arc, <0 = right arc
 * @return Filtered gray value (valid pattern or 0x00)
 */
uint8_t nav_filter_line_by_dir(uint8_t gray, float direction) {
    /* No valid data — pass through */
    if (gray == 0x00 || gray == 0xFF) return gray;

    if (direction > 0.5f) {
        /* LEFT arc: scan from rightmost sensor (bit0) upward.
                    左弧：从最右侧传感器向左侧扫描 */
        for (int i = NAV_SENSOR_BIT_RIGHT; i <= NAV_SENSOR_BIT_LEFT; i++) {
            if (gray & (1 << i)) {
                uint8_t result = 0;
                /* Collect contiguous dark bits (the real line) */
                while (i <= NAV_SENSOR_BIT_LEFT && (gray & (1 << i))) {
                    result |= (1 << i);
                    i++;
                }
                return result;
            }
        }
    } else if (direction < -0.5f) {
        /* RIGHT arc: scan from leftmost sensor (bit7) downward.
                     右弧：从最左侧传感器向右侧扫描 */
        for (int i = NAV_SENSOR_BIT_LEFT; i >= NAV_SENSOR_BIT_RIGHT; i--) {
            if (gray & (1 << i)) {
                uint8_t result = 0;
                while (i >= NAV_SENSOR_BIT_RIGHT && (gray & (1 << i))) {
                    result |= (1 << i);
                    i--;
                }
                return result;
            }
        }
    }
    return gray; /* direction near 0: no filtering, straight-line fallback */
}
