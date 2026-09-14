cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_kinematics CXX)       # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# 库本体 —— 唯一的**对外产物**
#
# 消费方只需要一行就能用上这个库：
#     target_link_libraries(<你的 target> PRIVATE kinematics)
# 即可拿到 inc/ 的头文件路径要求（INTERFACE 库不产生 .o）
# ===========================================================================
add_library(kinematics INTERFACE)
target_include_directories(kinematics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# ===========================================================================
# ⚠ 两种构建模式（统一约定 LUNOKHOD_DEV_BUILD）
#
#   未定义 → 跟随“我是不是顶层项目”：
#              单独构建          = 开发 → 生成测试/示例/工具
#              被别的工程 include = 消费 → 只出库
#   已定义 → 用给定值（lunokhod 根聚合器会设 ON）
#
# 为什么不用单纯的 PROJECT_IS_TOP_LEVEL：根聚合器 add_subdirectory 各组件时，
# 组件的 PROJECT_IS_TOP_LEVEL 会是 FALSE，那样聚合构建反而跑不了测试。
# ===========================================================================
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    message(STATUS "kinematics: 库模式 -> 只提供 kinematics（不生成测试/示例/工具）")
    return()
endif()

message(STATUS "kinematics: 开发模式 -> 生成测试 / 示例 / 工具")

# ---------------------------------------------------------------------------
# ↓ 以下全部是【开发期产物】，不属于库的对外接口
# ---------------------------------------------------------------------------

# 测试注册（2026-09-13 补：原 CMakeLists 没有 enable_testing/add_test，
# ctest 报 "No tests were found"，"全绿"实际靠手跑可执行文件）
enable_testing()

# Test: Individual executable with main
add_executable(test_kinematics test/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE kinematics)
target_compile_options(test_kinematics PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_kinematics COMMAND test_kinematics)

# Example: another individual executable file with main
add_executable(example_diff examples/example.cpp)
target_link_libraries(example_diff PRIVATE kinematics)
target_compile_options(example_diff PRIVATE -Wall -Wextra -Werror)
add_test(NAME example_diff COMMAND example_diff)

# L3 仿真 demo（三底盘 2D 位姿积分）
add_executable(simulation_demo examples/simulation_demo.cpp)
target_link_libraries(simulation_demo PRIVATE kinematics)
target_compile_options(simulation_demo PRIVATE -Wall -Wextra -Werror)
add_test(NAME simulation_demo COMMAND simulation_demo)

# ---------------------------------------------------------------------------
# odometry：源码 inc/odometry.hpp 由作者手写（AGENTS.md：AI 禁改 inc/）。
# 头文件未落地前不注册 target —— 保证仓库始终"全绿"。
# 金标重生：python3 test/tools/gen_odometry_golden.py
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/inc/odometry.hpp)
    add_executable(test_odometry test/test_odometry.cpp)
    target_link_libraries(test_odometry PRIVATE kinematics)
    target_compile_options(test_odometry PRIVATE -Wall -Wextra -Werror)
    add_test(NAME test_odometry COMMAND test_odometry)

    # 开发 / 分析工具（**不是**库示例）：仿真跑一遍 + 导 CSV，供 tools/plot_odometry.py 画图
    # 用途：打滑检测的原料采集、底盘建模、复盘分析（见 docs/TODO.md P19）
    add_executable(odometry_demo tools/odometry_demo.cpp)
    target_link_libraries(odometry_demo PRIVATE kinematics)
    target_compile_options(odometry_demo PRIVATE -Wall -Wextra -Werror)
    message(STATUS "kinematics: 已注册 test_odometry + odometry_demo")
else()
    message(STATUS "kinematics: 尚无 inc/odometry.hpp -> 跳过 odometry 相关 target")
endif()
