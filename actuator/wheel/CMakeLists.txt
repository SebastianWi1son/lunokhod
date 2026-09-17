cmake_minimum_required(VERSION 3.28)
project(lunokhod_wheel CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# 库本体 —— 对外产物
#   消费方： target_link_libraries(<你的 target> PRIVATE wheel)
# ===========================================================================
# 算法原语（pid/lpf/ramp/smooth_planner）自 v0.1.1 起来自上游库 ctlkit（vendor，见 third_party/ctlkit/VERSION）
add_library(wheel STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ctlkit/src/lpf.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ctlkit/src/ramp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ctlkit/src/smooth_planner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ctlkit/src/pid.cpp
        src/wheel.cpp)
target_include_directories(wheel PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc                              # 本组件头（含 4 个转发头）
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ctlkit/inc)    # 上游库头（ctl/...）
target_compile_options(wheel PRIVATE -Wall -Wextra -Werror)

# ── 两种构建模式（见 kinematics/CMakeLists.txt 的说明）──
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    message(STATUS "wheel: 库模式 -> 只提供 wheel")
    return()
endif()

message(STATUS "wheel: 开发模式 -> 生成测试 / 示例")
enable_testing()

# 测试
add_executable(test_wheel test/test_wheel.cpp)
target_link_libraries(test_wheel PRIVATE wheel)
target_compile_options(test_wheel PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_wheel COMMAND test_wheel)

# 示例（链式：含电机模型仿真）
add_executable(example_wheel examples/example_wheel.cpp)
target_link_libraries(example_wheel PRIVATE wheel)
target_compile_options(example_wheel PRIVATE -Wall -Wextra -Werror)
add_test(NAME example_wheel COMMAND example_wheel)
