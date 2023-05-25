cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(rp2040-can-mcp2515 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#list(APPEND PICO_CONFIG_HEADER_FILES ../../../../adafruit_feather_can_rp2040.h)
list(APPEND PICO_CONFIG_HEADER_FILES ${CUSTOM_BOARD_CONFIG_HEADER_FILES})

pico_sdk_init()

add_executable(rp2040-can-mcp2515
  main.c
  usb_descriptors.c
)
target_include_directories(rp2040-can-mcp2515 PRIVATE .)
target_link_libraries(rp2040-can-mcp2515 pico_stdlib hardware_spi tinyusb_device tinyusb_board pico_multicore hardware_pio)
pico_add_extra_outputs(rp2040-can-mcp2515)
pico_generate_pio_header(rp2040-can-mcp2515 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
