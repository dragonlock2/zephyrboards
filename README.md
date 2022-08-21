# zephyrboards
Zephyr RTOS board definitions for extra boards and drivers

## Setup

Follow https://docs.zephyrproject.org/latest/getting_started/index.html to get dependencies installed. Then create a new workspace using this repo.

    west init -m https://github.com/dragonlock2/zephyrboards.git <workspace name>
    cd <workspace name>
    west update

## Boards

| name | link | notes |
| ---- | ---- | ----- |
| k66f_breakout | https://matthewtran.dev/2021/08/k66f-breakout/ | |
| nogusb        | https://github.com/dragonlock2/kicadboards/tree/main/projects/NOGUSB | has MCUboot config |
| LPC845-BRK | https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc800-cortex-m0-plus-/lpc845-breakout-board-for-lpc84x-family-mcus:LPC845-BRK | basic uart, gpio working |
| leveler | https://github.com/berkeleyauv/electrical_training | |
| 6wire | https://github.com/dragonlock2/kicadboards/tree/main/projects/6wire | |
| efm32wg | https://github.com/dragonlock2/kicadboards/tree/main/breakouts/efm32wg | leuart, spi not working |

## Drivers

| name | notes |
| ---- | ----- |
| lin-uart | LIN API also created |
