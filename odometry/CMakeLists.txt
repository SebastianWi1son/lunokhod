cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_odometry CXX)         # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# 依赖 contracts（契约唯一来源）
#
# 优先用【消费方已经引入】的 target；没有才回退到本地相对路径。
# 为什么这么写：如果这里无条件 add_subdirectory，那么同时引 contracts 和
# odometry 的工程会把 contracts 添加两次 → CMake 报 target 重名。
# 这个写法同时支持"独立构建"与"被别的工程引入"两种用法。
# ===========================================================================
if(NOT TARGET contracts)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../contracts/CMakeLists.txt)
        add_subdirectory(../contracts ${CMAKE_BINARY_DIR}/_ext/contracts)
        message(STATUS "odometry: 独立构建 -> 自动引入本地 contracts")
    else()
        message(FATAL_ERROR
            "odometry 需要 target `contracts`。\n"
            "请在使用方工程里先 add_subdirectory(<lunokhod>/contracts ...) 或 FetchContent。")
    endif()
endif()

# ===========================================================================
# 库本体 —— 唯一的**对外产物**
#
# 消费方只需要一行就能用上这个库：
#     target_link_libraries(<你的 target> PRIVATE odometry)
# 即可拿到 inc/ 的头文件路径要求（INTERFACE 库不产生 .o）
# ===========================================================================
add_library(odometry INTERFACE)
target_include_directories(odometry INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(odometry INTERFACE contracts)

# ===========================================================================
# 两种构建模式（统一约定 LUNOKHOD_DEV_BUILD，说明见 kinematics/CMakeLists.txt）
#
#   未定义 → 跟随"我是不是顶层项目"：
#              单独构建          = 开发 → 生成测试/工具
#              被别的工程 include = 消费 → 只出库
#   已定义 → 用给定值（lunokhod 根聚合器会设 ON）
# ===========================================================================
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    message(STATUS "odometry: 库模式 -> 只提供 odometry（不生成测试/工具）")
    return()
endif()

message(STATUS "odometry: 开发模式 -> 生成测试 / 工具")

# ---------------------------------------------------------------------------
# ↓ 以下全部是【开发期产物】，不属于库的对外接口
# ---------------------------------------------------------------------------

enable_testing()

# Test: Individual executable with main
# 金标重生：python3 test/tools/gen_odometry_golden.py
add_executable(test_odometry test/test_odometry.cpp)
target_link_libraries(test_odometry PRIVATE odometry)
target_compile_options(test_odometry PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_odometry COMMAND test_odometry)

# 开发 / 分析工具（**不是**库示例）：仿真跑一遍 + 导 CSV，供 tools/plot_odometry.py 画图
# 用途：打滑检测的原料采集、底盘建模、复盘分析（见 docs/TODO.md P19）
add_executable(odometry_demo tools/odometry_demo.cpp)
target_link_libraries(odometry_demo PRIVATE odometry)
target_compile_options(odometry_demo PRIVATE -Wall -Wextra -Werror)
