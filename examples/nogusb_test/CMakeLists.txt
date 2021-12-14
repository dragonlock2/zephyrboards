cmake_minimum_required(VERSION 3.13.1)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../)

set(DTC_OVERLAY_FILE nogusb.overlay)
set(BOARD nogusb)

find_package(Zephyr)
project(nogusb_test)

target_sources(app PRIVATE src/main.c)
