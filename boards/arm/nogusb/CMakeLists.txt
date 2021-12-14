cmake_minimum_required(VERSION 3.13.1)

# hacky workaround to apply custom patch
add_custom_target(
    apply_patch
    COMMAND git apply ${BOARD_DIR}/nogusb.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
)
add_dependencies(app apply_patch)

add_custom_target(
    remove_patch ALL
    COMMAND git apply -R ${BOARD_DIR}/nogusb.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
    DEPENDS zephyr_final
)
