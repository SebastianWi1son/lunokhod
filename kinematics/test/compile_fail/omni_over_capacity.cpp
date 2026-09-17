// 反例编译测试 —— 本文件**必须编不过**（由 kinematics/CMakeLists.txt 的 try_compile 检查）。
// 它故意把轮数写成 7：超过 WheelSpeeds 的契约容量 6，应被 OmniDrive 的 static_assert 挡住。
// 为什么要有这个文件：让"守卫还在"这件事也变成**机器能验证**的事实 ——
// 谁把 static_assert 删了/放宽了，这里就会编过 → cmake 配置阶段直接 FATAL_ERROR（改坏必红）。
#include "chassis.hpp"

int main() {
    lunokhod::kinematics::OmniDrive<7> over(0.15f, 0.0f, 0.03f);   // 7 > 契约容量 6
    (void)over;
    return 0;
}
