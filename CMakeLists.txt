cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build configuration" FORCE)
endif()

project(stm32_omni_chassis LANGUAGES C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${PROJECT_NAME})

# CubeMX owns peripheral initialization and HAL/CMSIS dependencies.
add_subdirectory(cmake/stm32cubemx)

# Application modules are maintained separately from generated sources.
target_sources(${PROJECT_NAME} PRIVATE
    Core/APP/Encoder/encoder.c
    Core/APP/JY901P/jy901p.c
    Core/APP/Motor/motor.c
    Core/APP/Omni/Omni.c
    Core/APP/PID/pid.c
    Core/APP/Remote/remote_control.c
    Core/APP/Safety/motor_safety.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    Core/APP/Encoder
    Core/APP/JY901P
    Core/APP/Motor
    Core/APP/Omni
    Core/APP/PID
    Core/APP/Remote
    Core/APP/Safety
)

# Match the plain target_link_libraries signature used by CubeMX.
target_link_libraries(${PROJECT_NAME} stm32cubemx)
