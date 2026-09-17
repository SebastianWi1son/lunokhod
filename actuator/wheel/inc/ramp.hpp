#pragma once

// ctlkit-forwarder —— 机器可读标记：本文件是转发头，不是上游副本（校验脚本据此跳过逐字比对）
// 算法原语的上游是 ctlkit —— vendor 在 third_party/ctlkit/（VERSION 记来源 sha）。
// 保留本路径与**全局名**：本组件历史如此（PID/LPF/Ramp/SmoothPlanner 一直放在全局命名空间，
// wheel.hpp / chassis_loop / fw_poc 都直接写 PID、PIDConfig）→ 调用点零改动。
// 血缘：本组件即这些原语的血缘源头（ctlkit 收录时来源标注为 lunokhod/actuator/wheel）。
// 行为契约见上游库 spec（ctlkit 仓的 docs/spec/，未随 vendor 拷贝）。

#include "ctl/ramp.hpp"

using ctl::Ramp;
