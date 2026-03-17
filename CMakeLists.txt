cmake_minimum_required(VERSION 3.16)
project(i2c_lib VERSION 2.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)

# ---- Library ----
add_library(i2c
    src/i2c.c
    drivers/i2c_devices.c
)

target_include_directories(i2c PUBLIC
    include
    drivers
)

# ---- Example ----
add_executable(example_all_devices
    examples/example_all_devices.c
)

target_link_libraries(example_all_devices PRIVATE i2c)

# ---- Compiler warnings ----
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(i2c PRIVATE -Wall -Wextra -Wpedantic)
endif()
