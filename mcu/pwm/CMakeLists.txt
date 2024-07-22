cmake_minimum_required(VERSION 3.13)

# Pull in PICO SDK

include(pico_sdk_import.cmake)

project(pwm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# initialize SDK
pico_sdk_init()

add_executable(pwm
        pwm.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(pwm pico_stdlib hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(pwm)

