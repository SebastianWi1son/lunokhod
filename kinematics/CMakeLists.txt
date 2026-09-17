cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_kinematics CXX)       # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# 依赖 contracts（契约唯一来源）
#
# 优先用【消费方已经引入】的 target；没有才回退到本地相对路径。
# 为什么这么写：如果这里无条件 add_subdirectory，那么同时引 contracts 和
# kinematics 的工程会把 contracts 添加两次 → CMake 报 target 重名。
# ===========================================================================
if(NOT TARGET contracts)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../contracts/CMakeLists.txt)
        add_subdirectory(../contracts ${CMAKE_BINARY_DIR}/_ext/contracts)
        message(STATUS "kinematics: 独立构建 -> 自动引入本地 contracts")
    else()
        message(FATAL_ERROR
            "kinematics 需要 target `contracts`。\n"
            "请在使用方工程里先 add_subdirectory(<lunokhod>/contracts ...) 或 FetchContent。")
    endif()
endif()

# ===========================================================================
# 库本体 —— 唯一的**对外产物**
#
# 消费方只需要一行就能用上这个库：
#     target_link_libraries(<你的 target> PRIVATE kinematics)
# 即可拿到 inc/ 的头文件路径要求（INTERFACE 库不产生 .o）
# ===========================================================================
add_library(kinematics INTERFACE)
target_include_directories(kinematics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(kinematics INTERFACE contracts)

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
# 反例编译测试：非法轮数**必须编不过**（改坏必红）
#
# 判据：`OmniDrive<7>` 若编过了，说明 drive_omni.hpp 里的 static_assert 被删/放宽了
#       → 配置阶段直接 FATAL_ERROR（本地与 CI 一起红），而不是静默退化成运行期越界。
# 放在【开发模式】段内：消费方（库模式）不付这笔配置开销。
# ---------------------------------------------------------------------------
# 上下界各一个反例：超上界 / 低于下界。两个都必须编不过。
foreach(_case IN ITEMS omni_over_capacity omni_under_minimum)
    try_compile(_neg_${_case}_compiled
        ${CMAKE_BINARY_DIR}/_neg_compile/${_case}
        SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/compile_fail/${_case}.cpp
        CMAKE_FLAGS
            "-DCMAKE_CXX_STANDARD=17"
            "-DINCLUDE_DIRECTORIES=${CMAKE_CURRENT_SOURCE_DIR}/inc;${CMAKE_CURRENT_SOURCE_DIR}/../contracts/inc"
        OUTPUT_VARIABLE _neg_out)
    if(_neg_${_case}_compiled)
        message(FATAL_ERROR
            "反例编译测试失败：test/compile_fail/${_case}.cpp 本应编不过，却编过了 ——\n"
            "轮数边界守卫（OmniDrive 的 static_assert / kMinWheels / kMaxWheels）被删或放宽了。")
    endif()
endforeach()
message(STATUS "kinematics: 反例编译测试通过（超上界 + 低于下界两个非法轮数都编不过）")

# ---------------------------------------------------------------------------
# odometry 已独立成组件（`../odometry/`），不再在本库注册。
# 它的测试与工具现在跟着它自己走，见 odometry/CMakeLists.txt。
# ---------------------------------------------------------------------------
