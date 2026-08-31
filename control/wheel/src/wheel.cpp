#include "wheel.hpp"

Wheel::Wheel(uint8_t motor_id,
    const SmoothPlannerConfig &sp_cfg,
    const PIDConfig &pid,
    MeasureSpeedFn measure_speed,
    SetPwmFn set_pwm)

    : motor_id_(motor_id), speed_cmd_(0.0f), speed_cur_(0.0f),
      planner_(sp_cfg.max_rate_, sp_cfg.Tf_), pid_(pid),
      measure_speed_(measure_speed), set_pwm_(set_pwm) {}


void Wheel::set_cmd(float speed_cmd) { speed_cmd_ = speed_cmd; }

void Wheel::update(float dt) {
    if (measure_speed_) { speed_cur_ =  measure_speed_(motor_id_, dt); }    // get wheel speed
    float planned_cmd = planner_.calc(speed_cmd_, dt);                      // smooth planner
    float out_pwm = pid_.calc(planned_cmd, speed_cur_, dt);         // wheel speed loop
    if (set_pwm_) { set_pwm_(motor_id_, out_pwm); }                         // set pwm
}

void Wheel::stop() {
    speed_cmd_ = 0.0f;
    pid_.reset();
    planner_.reset();
    if (set_pwm_) { set_pwm_(motor_id_, 0.0f); }
}

float Wheel::get_speed() const { return speed_cur_; }
