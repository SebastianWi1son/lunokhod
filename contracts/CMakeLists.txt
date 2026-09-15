cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_contracts CXX)        # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# 库本体 —— 唯一的**对外产物**
#
# 契约层：跨组件共享的数据类型（`Twist` 输入 / `WheelSpeeds` 输出 / `Pose` 位姿）。
#
# 它是最底层 —— 只依赖 `<cstdint>`，**不依赖本仓任何别的库**，谁都可以依赖它。
# 消费方只需要一行：
#     target_link_libraries(<你的 target> PRIVATE contracts)
# ===========================================================================
add_library(contracts INTERFACE)
target_include_directories(contracts INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# ===========================================================================
# 为什么这里【没有】LUNOKHOD_DEV_BUILD 两种构建模式的守卫（别的组件都有）：
#
#   那个守卫的用途是"被引入时不生成测试/示例/工具"。
#   本组件没有测试 / 示例 / 工具 —— 只有一份 18 行的纯类型定义。
#   没有开发期产物，也就没有需要守卫的东西。
#
#   将来若给 contracts 加了测试，必须把守卫照抄回来。
# ===========================================================================
