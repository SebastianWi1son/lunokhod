cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_twist_acc_limiter CXX)       # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖 contracts（契约唯一来源）
#
# 为什么是 contracts 而不是 kinematics：本组件只用到 `Twist` 这一个契约类型，
# 与运动学（正/逆解）毫无关系。以前它依赖 kinematics，那是一条**语义不实**的边。
#
# 优先用【消费方已经引入】的 target；没有才回退到本地相对路径。
# 为什么这么写：如果这里无条件 add_subdirectory，那么固件工程同时引 contracts
# 和 twist_acc_limiter 时，contracts 会被添加两次 → CMake 报 target 重名。
# 这个写法同时支持“独立构建”与“被别的工程引入”两种用法。
if(NOT TARGET contracts)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../contracts/CMakeLists.txt)
        add_subdirectory(../../contracts ${CMAKE_BINARY_DIR}/_ext/contracts)
        message(STATUS "twist_acc_limiter: 独立构建 -> 自动引入本地 contracts")
    else()
        message(FATAL_ERROR
            "twist_acc_limiter 需要 target `contracts`。\n"
            "请在使用方工程里先 add_subdirectory(<lunokhod>/contracts ...) 或 FetchContent。")
    endif()
endif()

# ===========================================================================
# 库本体 —— 对外产物
#   消费方： target_link_libraries(<你的 target> PRIVATE twist_acc_limiter)
# ===========================================================================
add_library(twist_acc_limiter STATIC src/twist_acc_limiter.cpp)
target_include_directories(twist_acc_limiter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(twist_acc_limiter PUBLIC contracts)
target_compile_options(twist_acc_limiter PRIVATE -Wall -Wextra -Werror)

# ── 两种构建模式（见 kinematics/CMakeLists.txt 的说明）──
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    message(STATUS "twist_acc_limiter: 库模式 -> 只提供 twist_acc_limiter（只依赖 contracts）")
    return()
endif()

message(STATUS "twist_acc_limiter: 开发模式 -> 生成测试 / 示例")
enable_testing()

# ── 开发期独有：示例演示的是【链式集成】，所以它多一层对 kinematics 的依赖 ──
#   库本体 / 测试都只用 contracts；只有 examples/ 里用了 DiffDrive（见其 include）。
#   把这条边放在开发期，库模式下的依赖就保持干净。
if(NOT TARGET kinematics)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../kinematics/CMakeLists.txt)
        add_subdirectory(../../kinematics ${CMAKE_BINARY_DIR}/_ext/kinematics)
        message(STATUS "twist_acc_limiter: 示例需要 -> 自动引入本地 kinematics")
    else()
        message(FATAL_ERROR
            "twist_acc_limiter 的示例需要 target `kinematics`。\n"
            "请在使用方工程里先 add_subdirectory(<lunokhod>/kinematics ...) 或 FetchContent。")
    endif()
endif()

# Test: Individual executable with main
add_executable(test_twist_acc_limiter test/test_twist_acc_limiter.cpp)
target_link_libraries(test_twist_acc_limiter PRIVATE twist_acc_limiter)
target_compile_options(test_twist_acc_limiter PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_twist_acc_limiter COMMAND test_twist_acc_limiter)

# Example: another individual executable file with main
# ⚠ 这个示例额外依赖 kinematics（它演示的是“限幅 → DiffDrive 逆解”的链式集成）
add_executable(example_twist_acc_limiter examples/example_twist_acc_limiter.cpp)
target_link_libraries(example_twist_acc_limiter PRIVATE twist_acc_limiter kinematics)
target_compile_options(example_twist_acc_limiter PRIVATE -Wall -Wextra -Werror)
add_test(NAME example_twist_acc_limiter COMMAND example_twist_acc_limiter)
