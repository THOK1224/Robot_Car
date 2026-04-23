cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Robot_Car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Middleware Layer
    0-Middleware_Layer/Pid_Controller/algorithm_pid.c
    0-Middleware_Layer/Kalman_Fliter/algorithm_ekf.c
    0-Middleware_Layer/Kalman_Fliter/algorithm_kf.c
    0-Middleware_Layer/Message_Center/message_center.c
    0-Middleware_Layer/Error_System/src/error_handler.c
    0-Middleware_Layer/Error_System/src/error_port.c
    0-Middleware_Layer/Crc/crc_referee.c

    # Hardware Layer
    1-Hardware_Layer/bsp_dwt/bsp_dwt.c
    1-Hardware_Layer/bsp_usart/bsp_usart.c
    1-Hardware_Layer/bsp_wdg/bsp_wdg.c
    1-Hardware_Layer/bsp_can/bsp_can.c
    1-Hardware_Layer/bsp_iic/bsp_iic.c

    # Hardware Driver Layer
    2-Hardware_Driver_Layer/Motor/motor.c
    2-Hardware_Driver_Layer/Ultrasonic/ultrasonic.c
    2-Hardware_Driver_Layer/PS2_Driver/ps2_driver.c
    2-Hardware_Driver_Layer/Yabo_Motor_Driver/yabo_motor_driver.c
    2-Hardware_Driver_Layer/IMU_Driver/dm_imu.c

    # Function Layer
    3-Function_Layer/Chassis/chassis.c
    3-Function_Layer/PS2_Control/ps2_control.c
    3-Function_Layer/Sensor/sensor.c

    # Application Layer
    4-Application_Layer/Robot_Task/robot_task.c
    4-Application_Layer/Test_task/test_task.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Middleware Layer
    0-Middleware_Layer/Pid_Controller
    0-Middleware_Layer/Kalman_Fliter
    0-Middleware_Layer/Math_lib
    0-Middleware_Layer/Message_Center
    0-Middleware_Layer/Error_System/inc
    0-Middleware_Layer/Crc

    # Hardware Layer
    1-Hardware_Layer/bsp_dwt
    1-Hardware_Layer/bsp_usart
    1-Hardware_Layer/bsp_wdg
    1-Hardware_Layer/bsp_can
    1-Hardware_Layer/bsp_iic

    # Hardware Driver Layer
    2-Hardware_Driver_Layer/Motor
    2-Hardware_Driver_Layer/Ultrasonic
    2-Hardware_Driver_Layer/PS2_Driver
    2-Hardware_Driver_Layer/Yabo_Motor_Driver
    2-Hardware_Driver_Layer/IMU_Driver

    # Function Layer
    3-Function_Layer
    3-Function_Layer/Chassis
    3-Function_Layer/PS2_Control
    3-Function_Layer/Sensor

    # Application Layer
    4-Application_Layer/Robot_Task
    4-Application_Layer/Test_task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
