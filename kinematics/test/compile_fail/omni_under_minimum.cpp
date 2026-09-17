// 反例编译测试 —— 本文件**必须编不过**（由 kinematics/CMakeLists.txt 的 try_compile 检查）。
// 它把轮数写成 2：轮数下界是 3（N=2 时 J 的秩只有 2，vx 不可控也不可观测），应被 static_assert 挡住。
// 与 omni_over_capacity.cpp 一起把**上下界都钉住**：谁把 kMinWheels/kMaxWheels 放宽或删掉 assert，
// 这里就会编过 → cmake 配置阶段直接 FATAL_ERROR（改坏必红）。
#include "chassis.hpp"

int main() {
    lunokhod::kinematics::OmniDrive<2> under(0.15f, 0.0f, 0.03f);   // 2 < kMinWheels
    (void)under;
    return 0;
}
