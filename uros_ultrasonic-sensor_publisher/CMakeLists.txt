cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
add_executable(main
        publisher.cpp
        pico_uart_transport/pico_uart_transport.c
        GPIOInputMgr.cpp
        GPIOObserver.cpp    
        HCSR04Range.cpp 
        distance_sensor.cpp
        )

target_link_libraries(main
    pico_stdlib
    microros
    hardware_pio
    hardware_irq
    pico_multicore
)

target_include_directories(main PUBLIC
    libmicroros/include
    pico_uart_transport
)
pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/sensor.pio)

pico_add_extra_outputs(main)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(main)
