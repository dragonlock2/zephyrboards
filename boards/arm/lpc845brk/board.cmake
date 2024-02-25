board_runner_args(pyocd "--target=lpc845")

# not sure why, but sometimes need to run and cancel openocd before pyocd works
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
