cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(
    libmicroros
)
add_executable(main
        main.cpp
        pico_uart_transport/pico_uart_transport.c
        )

add_subdirectory(
	PAA5160E1_Pico_Library
)
target_link_libraries(main
    pico_stdlib # Standard library for Pico
    microros # micro-ROS library
    hardware_i2c
    Qwiic_OTOS_Library
    hardware_pwm
)

target_include_directories(main PUBLIC
    libmicroros/include # Include headers for micro-ROS
    pico_uart_transport # Include headers specific to Pico UART transport
    libmicroros/include/nav_msgs # Include nav_msgs
    libmicroros/include/geometry_msgs # Include geometry_msgs
    libmicroros/include/sensor_msgs # Include sensor_msgs
    libmicroros/include/rosidl_typesupport_introspection_c
)

# Add extra outputs for the "main" executable (binary files, UF2, etc.)
pico_add_extra_outputs(main)

# Set compilation flags to optimize the binary size
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Enable USB output and disable UART output for standard I/O
pico_enable_stdio_usb(main 1) # Enable USB output, necessary for using picotool for loading
pico_enable_stdio_uart(main 0) # Disable UART output

# Add compilation definitions to configure carriage return and line feed (CRLF) support
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0) # Disable CRLF support for UART
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0) # Disable CRLF support for stdio
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0) # Set the default CRLF conversion to 0 (no conversion)

# Add extra outputs again to ensure all configurations are accounted for
pico_add_extra_outputs(main)
