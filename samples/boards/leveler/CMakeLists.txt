cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE leveler.overlay)
set(BOARD leveler)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(leveler)

target_sources(app PRIVATE src/main.c)
