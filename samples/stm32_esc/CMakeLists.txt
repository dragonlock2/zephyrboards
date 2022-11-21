cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32_esc)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32_esc)

target_sources(app PRIVATE src/main.c)
