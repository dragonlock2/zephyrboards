cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE efm32wg.overlay)
set(BOARD efm32wg)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(efm32wg_test)

target_sources(app PRIVATE src/main.c)
