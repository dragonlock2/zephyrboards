# SPDX-License-Identifier: Apache-2.0
board_runner_args(stm32cubeprogrammer "--port=swd")
board_runner_args(jlink "--device=STM32G0B1RE" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
