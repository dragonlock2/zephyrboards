cmake_minimum_required(VERSION 3.20.0)

include_directories(src) # override header :)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rubberducky)

target_include_directories(app PRIVATE
    inc
)

target_sources(app PRIVATE
    src/main.cpp
    src/error.cpp
    src/hid.cpp
    src/payload.cpp
    src/rgb.cpp
)
