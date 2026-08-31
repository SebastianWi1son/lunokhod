cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_twist_acc_limiter CXX)       # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖 kinematics（INTERFACE target，自动传播 include 路径；契约唯一来源）
add_subdirectory(../../kinematics ${CMAKE_BINARY_DIR}/kinematics)

# 库 STATIC: 声明在 .hpp、实现放 .cpp（声明定义分离）
add_library(twist_acc_limiter STATIC src/twist_acc_limiter.cpp)
target_include_directories(twist_acc_limiter PUBLIC ${PROJECT_SOURCE_DIR}/inc )
target_link_libraries(twist_acc_limiter PUBLIC kinematics)
target_compile_options(twist_acc_limiter PRIVATE -Wall -Wextra -Werror)

# Test: Individual executable with main
add_executable(test_twist_acc_limiter test/test_twist_acc_limiter.cpp)
target_link_libraries(test_twist_acc_limiter PRIVATE twist_acc_limiter)
target_compile_options(test_twist_acc_limiter PRIVATE -Wall -Wextra -Werror)

# Example: another individual executable file with main
add_executable(example_twist_acc_limiter examples/example_twist_acc_limiter.cpp)
target_link_libraries(example_twist_acc_limiter PRIVATE twist_acc_limiter)
target_compile_options(example_twist_acc_limiter PRIVATE -Wall -Wextra -Werror)
