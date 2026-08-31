cmake_minimum_required(VERSION 3.28)    # CMake Minimum

project(lunokhod_kinematics CXX)       # CMake project

set(CMAKE_CXX_STANDARD 17)              # CPP STANDARD
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 库 INTERFACE: header-only, dont generate .o file
add_library(kinematics INTERFACE)
target_include_directories(kinematics INTERFACE ${PROJECT_SOURCE_DIR}/inc )

# Test: Individual executable with main
add_executable(test_kinematics test/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE kinematics)
target_compile_options(test_kinematics PRIVATE -Wall -Wextra -Werror)

# Example: another individual executable file with main
add_executable(example_diff examples/example.cpp)
target_link_libraries(example_diff PRIVATE kinematics)

