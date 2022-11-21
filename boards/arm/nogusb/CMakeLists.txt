cmake_minimum_required(VERSION 3.20.0)

add_custom_target(
    apply_patch
    COMMAND git apply ${BOARD_DIR}/nogusb.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
)
add_dependencies(version_h apply_patch)

add_custom_target(
    remove_patch ALL
    COMMAND git apply -R ${BOARD_DIR}/nogusb.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
    DEPENDS zephyr_final
)
