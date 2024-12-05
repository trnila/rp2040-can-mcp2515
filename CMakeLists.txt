cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(rp2040-can-mcp2515 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

function(add_board BOARD)
  add_executable(gs_usb_${BOARD}
    main.c
    usb_descriptors.c
  )
  target_include_directories(gs_usb_${BOARD} PRIVATE .)
  target_link_libraries(gs_usb_${BOARD} pico_stdlib hardware_spi tinyusb_device tinyusb_board)
  target_compile_definitions(gs_usb_${BOARD} PRIVATE -DBOARD_${BOARD})
  pico_add_extra_outputs(gs_usb_${BOARD})
endfunction()

add_board(PICO_MCP2515_MODULE_8MHZ)
add_board(PICO_MCP2515_MODULE_20MHZ)
add_board(ADAFRUIT_CAN_FEATHER)
