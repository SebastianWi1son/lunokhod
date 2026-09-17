#pragma once

// ctlkit-forwarder —— 机器可读标记：本文件是转发头，不是上游副本（校验脚本据此跳过逐字比对）
// 算法原语的上游是 ctlkit —— vendor 在 third_party/ctlkit/（VERSION 记来源 sha）。
// 保留本路径；**类型收进 `namespace wheel`**（P29，2026-09-17）：谁 include 本头，只拿到
// `wheel::PID` 等 —— 不再往全局命名空间注入名字。调用点写 `wheel::PID`，或自己 `using wheel::PID`。
// 血缘：本组件即这些原语的血缘源头（ctlkit 收录时来源标注为 lunokhod/actuator/wheel）。
// 行为契约见上游库 spec（ctlkit 仓的 docs/spec/，未随 vendor 拷贝）。

#include "ctl/lpf.hpp"

namespace lunokhod::wheel {

using ctl::LPF;

}  // namespace lunokhod::wheel
