cmake_minimum_required(VERSION 3.20.0)

zephyr_library()
zephyr_library_sources(board.c)

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND lpc_checksum -f hex ${CMAKE_BINARY_DIR}/zephyr/${KERNEL_HEX_NAME}
    COMMAND lpc_checksum -f bin ${CMAKE_BINARY_DIR}/zephyr/${KERNEL_BIN_NAME}
)
