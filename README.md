# zephyrboards
Zephyr RTOS board definitions for extra boards.

## Setup

Follow https://docs.zephyrproject.org/latest/getting_started/index.html to get dependencies installed. Then create a new workspace using this repo.

    west init -m https://github.com/dragonlock2/zephyrboards.git <workspace name>
    cd <workspace name>
    west update

## Boards

| name | link | notes |
| ---- | ---- | ----- |
| k66f_breakout | https://matthewtran.dev/2021/08/k66f-breakout/ | |
| nogusb        | https://github.com/berkeleyauv/RoboSub_HW/tree/master/General/NOGUSB | |
| LPC845-BRK | https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc800-cortex-m0-plus-/lpc845-breakout-board-for-lpc84x-family-mcus:LPC845-BRK | very WIP |
