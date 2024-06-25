zephyr_library()
zephyr_library_sources(board.c)

# TODO can remove in 3.7.0
# patch to add USB HS support
add_custom_target(
    apply_zephyr_patch
    COMMAND git apply ${BOARD_DIR}/zephyr.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
)
add_dependencies(version_h apply_zephyr_patch)

add_custom_target(
    remove_zephyr_patch ALL
    COMMAND git apply -R ${BOARD_DIR}/zephyr.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
    DEPENDS zephyr_final
)
