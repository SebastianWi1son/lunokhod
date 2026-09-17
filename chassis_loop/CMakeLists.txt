cmake_minimum_required(VERSION 3.28)
project(lunokhod_chassis_loop CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖五个库：优先用消费方已引入的 target，没有才回退本地相对路径
# （理由与写法见 ../../odometry/CMakeLists.txt —— 无条件 add_subdirectory 会 target 重名）
if(NOT TARGET contracts)
    add_subdirectory(../contracts ${CMAKE_BINARY_DIR}/_ext/contracts)
endif()
foreach(dep kinematics odometry)
    if(NOT TARGET ${dep})
        add_subdirectory(../${dep} ${CMAKE_BINARY_DIR}/_ext/${dep})
    endif()
endforeach()
if(NOT TARGET wheel)
    add_subdirectory(../actuator/wheel ${CMAKE_BINARY_DIR}/_ext/wheel)
endif()
if(NOT TARGET twist_acc_limiter)
    add_subdirectory(../command/twist_acc_limiter ${CMAKE_BINARY_DIR}/_ext/twist_acc_limiter)
endif()

# 库本体：**INTERFACE** —— 全模板 + POD 配置，没有 .cpp（DESIGN.md §9 F2）
add_library(chassis_loop INTERFACE)
target_include_directories(chassis_loop INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(chassis_loop INTERFACE
        contracts kinematics odometry wheel twist_acc_limiter)

# 两种构建模式守卫（AGENTS.md §5.1：被引入时只出库）
if(NOT DEFINED LUNOKHOD_DEV_BUILD)
    set(LUNOKHOD_DEV_BUILD ${PROJECT_IS_TOP_LEVEL})
endif()
if(NOT LUNOKHOD_DEV_BUILD)
    return()
endif()

enable_testing()
add_executable(test_chassis_loop test/test_chassis_loop.cpp)
target_link_libraries(test_chassis_loop PRIVATE chassis_loop)
target_compile_options(test_chassis_loop PRIVATE -Wall -Wextra -Werror)
add_test(NAME test_chassis_loop COMMAND test_chassis_loop)