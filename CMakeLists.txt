cmake_minimum_required(VERSION 3.28)

project(lunokhod LANGUAGES CXX)

# ============================================================================
# lunokhod —— 底盘控制系统【库集合】
#
# ⚠ 本文件是【开发期聚合器】，**不是库的入口**。
# ============================================================================
#   开发者（你）
#       cmake -S lunokhod -B build && cmake --build build -j
#       ctest --test-dir build --output-on-failure
#       → 一键构建并测试所有组件
#
#   消费方（固件工程 / 别的项目）
#       **不要** include 本文件！按需引具体组件：
#           add_subdirectory(<lunokhod>/contracts                    ${CMAKE_BINARY_DIR}/_ext/contracts)
#           add_subdirectory(<lunokhod>/kinematics                   ${CMAKE_BINARY_DIR}/_ext/kinematics)
#           add_subdirectory(<lunokhod>/odometry                     ${CMAKE_BINARY_DIR}/_ext/odometry)
#           add_subdirectory(<lunokhod>/control/wheel                ${CMAKE_BINARY_DIR}/_ext/wheel)
#           add_subdirectory(<lunokhod>/control/twist_acc_limiter    ${CMAKE_BINARY_DIR}/_ext/tal)
#       组件会自动进入"库模式"：只出库，不出测试/示例/工具。
#       参考实现：~/Develop/Workspace/fw_poc/CMakeLists.txt
#
#   为什么要有这个聚合器：没有它，跑全部测试得进 5 个目录各配置一次。
#   为什么消费方不该用它：它会把 5 个组件的测试/示例/工具全部生成出来 ——
#       交叉编译固件时那是灾难。
# ============================================================================

# 两种构建模式的总开关（详见 kinematics/CMakeLists.txt 顶部说明）
#   未定义 → 跟随"我是不是顶层项目"
#   已定义 → 用给定值，并传给所有子组件
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
set(LUNOKHOD_DEV_BUILD ${LUNOKHOD_DEV_BUILD} CACHE BOOL
    "构建各组件自带的测试 / 示例 / 工具（库的消费方应设为 OFF）" FORCE)

enable_testing()

# 依赖顺序：contracts 必须最先（kinematics / odometry / twist_acc_limiter 都要它的契约）
add_subdirectory(contracts)
add_subdirectory(kinematics)
add_subdirectory(odometry)
add_subdirectory(control/wheel)
add_subdirectory(control/twist_acc_limiter)
# 装配层在最上层：它编排其余全部，所以必须放最后
add_subdirectory(chassis_loop)

message(STATUS "")
message(STATUS "lunokhod 开发构建 (LUNOKHOD_DEV_BUILD=${LUNOKHOD_DEV_BUILD})：")
message(STATUS "  库     contracts / kinematics / odometry / wheel / twist_acc_limiter / chassis_loop")
message(STATUS "  依赖   contracts ← 其余全部（单向，无环）")
message(STATUS "  全部测试  ctest --test-dir <build> --output-on-failure")
