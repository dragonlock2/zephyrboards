cmake_minimum_required(VERSION 3.20.0)

set(BASE ${CMAKE_CURRENT_SOURCE_DIR}/../../)

list(APPEND BOARD_ROOT ${BASE})
list(APPEND SOC_ROOT ${BASE})
list(APPEND DTS_ROOT ${BASE})

# add_subdirectory(${BASE}/drivers)

set(DTC_OVERLAY_FILE lpc845brk.overlay)
set(BOARD lpc845brk)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lpc845brk_test)

target_sources(app PRIVATE src/main.c)
