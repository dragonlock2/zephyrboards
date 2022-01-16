cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../)

set(DTC_OVERLAY_FILE nogusb.overlay)
set(BOARD nogusb)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nogusb_test)

target_sources(app PRIVATE src/main.c)
