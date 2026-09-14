cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_twist_acc_limiter CXX)       # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖 kinematics（契约唯一来源）
#
# 优先用【消费方已经引入】的 target；没有才回退到本地相对路径。
# 为什么这么写：如果这里无条件 add_subdirectory，那么固件工程同时引 kinematics
# 和 twist_acc_limiter 时，kinematics 会被添加两次 → CMake 报 target 重名。
# 这个写法同时支持“独立构建”与“被别的工程引入”两种用法。
if(NOT TARGET kinematics)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../kinematics/CMakeLists.txt)
        add_subdirectory(../../kinematics ${CMAKE_BINARY_DIR}/_ext/kinematics)
        message(STATUS "twist_acc_limiter: 独立构建 -> 自动引入本地 kinematics")
    else()
        message(FATAL_ERROR
            "twist_acc_limiter 需要 target `kinematics`。\n"
            "请在使用方工程里先 add_subdirectory(<lunokhod>/kinematics ...) 或 FetchContent。")
    endif()
endif()

# ===========================================================================
# 库本体 —— 对外产物
#   消费方： target_link_libraries(<你的 target> PRIVATE twist_acc_limiter)
# ===========================================================================
add_library(twist_acc_limiter STATIC src/twist_acc_limiter.cpp)
target_include_directories(twist_acc_limiter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(twist_acc_limiter PUBLIC kinematics)
target_compile_options(twist_acc_limiter PRIVATE -Wall -Wextra -Werror)

# ── 两种构建模式（见 kinematics/CMakeLists.txt 的说明）──
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    message(STATUS "twist_acc_limiter: 库模式 -> 只提供 twist_acc_limiter")
    return()
endif()

message(STATUS "twist_acc_limiter: 开发模式 -> 生成测试 / 示例")
enable_testing()

# Test: Individual executable with main
add_executable(test_twist_acc_limiter test/test_twist_acc_limiter.cpp)
target_link_libraries(test_twist_acc_limiter PRIVATE twist_acc_limiter)
target_compile_options(test_twist_acc_limiter PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_twist_acc_limiter COMMAND test_twist_acc_limiter)

# Example: another individual executable file with main
add_executable(example_twist_acc_limiter examples/example_twist_acc_limiter.cpp)
target_link_libraries(example_twist_acc_limiter PRIVATE twist_acc_limiter)
target_compile_options(example_twist_acc_limiter PRIVATE -Wall -Wextra -Werror)
add_test(NAME example_twist_acc_limiter COMMAND example_twist_acc_limiter)
