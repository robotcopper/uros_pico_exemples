# Set the minimum required version of CMake for this project
cmake_minimum_required(VERSION 3.13)

# Include the CMake file to import the Raspberry Pi Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Declare the project with its name and the languages used
project(main C CXX ASM)
set(CMAKE_C_STANDARD 11) # Set the C standard to use
set(CMAKE_CXX_STANDARD 17) # Set the C++ standard to use

# Initialize the Pico SDK
pico_sdk_init()

# Add the directory for libraries to search
link_directories(libmicroros)

# Add an executable named "main" including the specified source files
add_executable(main
        subscriber.cpp
        pico_uart_transport/pico_uart_transport.c
        )

# Link the necessary libraries to the "main" executable
target_link_libraries(main
    pico_stdlib # Standard library for Pico
    microros # micro-ROS library
)

# Specify the include directories for the "main" executable
target_include_directories(main PUBLIC
    libmicroros/include # Include headers for micro-ROS
    pico_uart_transport # Include headers specific to Pico UART transport
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

