#include "wheel.hpp"

namespace lunokhod::wheel {

Wheel::Wheel(uint8_t motor_id,
    const SmoothPlannerConfig &sp_cfg,
    const PIDConfig &pid,
    MeasureSpeedFn measure_speed,
    SetEffortFn    set_effort)

    : motor_id_(motor_id), speed_cmd_(0.0f), speed_cur_(0.0f), effort_(0),
      planner_(sp_cfg.max_rate_, sp_cfg.Tf_), pid_(pid),
      measure_speed_(measure_speed), set_effort_(set_effort) {}


void Wheel::set_cmd(float speed_cmd) { speed_cmd_ = speed_cmd; }

void Wheel::update(float dt) {
    if (measure_speed_) { speed_cur_ = measure_speed_(motor_id_, dt); }     // get wheel speed
    const float planned_cmd = planner_.calc(speed_cmd_, dt);                // smooth planner
    const float out_effort  = pid_.calc(planned_cmd, speed_cur_, dt);       // wheel speed loop
    effort_ = static_cast<int16_t>(out_effort);                             // 显式收窄（P23）
    if (set_effort_) { set_effort_(motor_id_, effort_); }                   // set effort
}

void Wheel::stop() {
    speed_cmd_ = 0.0f;
    pid_.reset();
    planner_.reset();
    effort_ = static_cast<int16_t>(0);
    if (set_effort_) { set_effort_(motor_id_, effort_); }
}

float   Wheel::get_speed() const { return speed_cur_; }
int16_t Wheel::effort()    const { return effort_; }

}  // namespace lunokhod::wheel
