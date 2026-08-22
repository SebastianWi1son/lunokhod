cmake_minimum_required(VERSION 3.28)
project(lunokhod_wheel CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 库 STATIC: 声明/定义分离
add_library(wheel STATIC
        src/lpf.cpp
        src/ramp.cpp
        src/smooth_planner.cpp
        src/pid.cpp
        src/wheel.cpp)
target_include_directories(wheel PUBLIC ${PROJECT_SOURCE_DIR}/inc)
target_compile_options(wheel PRIVATE -Wall -Wextra -Werror)

# 测试
add_executable(test_wheel test/test_wheel.cpp)
target_link_libraries(test_wheel PRIVATE wheel)
target_compile_options(test_wheel PRIVATE -Wall -Wextra -Werror)

# 示例（链式：含电机模型仿真）
add_executable(example_wheel
        examples/example_wheel.cpp
)
target_link_libraries(example_wheel PRIVATE wheel)
target_compile_options(example_wheel PRIVATE -Wall -Wextra -Werror)