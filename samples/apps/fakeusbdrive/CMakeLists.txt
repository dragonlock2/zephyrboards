cmake_minimum_required(VERSION 3.20.0)

set(BOARD leveler)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/fakedisk)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fakeusbdrive)

target_sources(app PRIVATE src/main.c)
