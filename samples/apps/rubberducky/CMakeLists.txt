cmake_minimum_required(VERSION 3.20.0)

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

add_custom_target(
    apply_rubberducky_patch
    COMMAND git apply ${CMAKE_CURRENT_LIST_DIR}/zephyr.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
)
add_dependencies(version_h apply_rubberducky_patch)

add_custom_target(
    remove_rubberducky_patch ALL
    COMMAND git apply -R ${CMAKE_CURRENT_LIST_DIR}/zephyr.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
    DEPENDS zephyr_final
)

# TODO use EDK from 3.7.0
# west build -t <llext target>
# will need to manually remove patches after
add_llext_target(example
    OUTPUT example.elf
    SOURCES payloads/example.cpp
)
