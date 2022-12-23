cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(rp2040-can-mcp2515 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040-can-mcp2515
  main.c
  usb_descriptors.c
)
pico_enable_stdio_usb(rp2040-can-mcp2515 1)
pico_enable_stdio_uart(rp2040-can-mcp2515 0)
pico_add_extra_outputs(rp2040-can-mcp2515)
target_include_directories(rp2040-can-mcp2515 PRIVATE .)
target_link_libraries(rp2040-can-mcp2515 pico_stdlib hardware_spi tinyusb_device tinyusb_board)
