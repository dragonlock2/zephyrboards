cmake_minimum_required(VERSION 3.13.1)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../)

set(DTC_OVERLAY_FILE nogusb.overlay)
set(BOARD nogusb)

find_package(Zephyr)
project(nogusb_test)

target_sources(app PRIVATE src/main.c)

# hacky workaround to apply custom patch
add_custom_target(
	apply_patch
	COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/nogusb.patch
	WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
)
add_dependencies(app apply_patch)

add_custom_target(
	remove_patch ALL
	COMMAND git apply -R ${CMAKE_CURRENT_SOURCE_DIR}/nogusb.patch
	WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
	DEPENDS zephyr_final
)
