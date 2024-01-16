# zephyrboards

Extra Zephyr RTOS board definitions, microcontroller support, drivers, and scripts.

## setup

Follow [Zephyr's Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html) to get dependencies installed. Then create a new workspace using this repo.

```
west init -m https://github.com/dragonlock2/zephyrboards.git <workspace name>
cd <workspace name>
west update
```

## boards

| name | notes |
| ---- | ----- |
| [k66f_breakout](https://matthewtran.dev/2021/08/k66f-breakout/) | |
| [nogusb](https://github.com/dragonlock2/kicadboards/tree/main/projects/NOGUSB) | MCUboot |
| [lpc845brk](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc800-cortex-m0-plus-/lpc845-breakout-board-for-lpc84x-family-mcus:LPC845-BRK) | |
| [leveler](https://github.com/berkeleyauv/electrical_training) | |
| [6wire](https://matthewtran.dev/2022/05/6wire/) | |
| [efm32wg](https://github.com/dragonlock2/kicadboards/tree/main/breakouts/efm32wg) | leuart, spi not working |
| [jabican_usb_pro](https://matthewtran.dev/2022/12/jabican-usb-pro/) | MCUboot |
| [stm32_esc](https://matthewtran.dev/2022/12/stm32-esc/) | |
| [lpc845_lin](https://github.com/dragonlock2/kicadboards/tree/main/breakouts/lpc845_lin) | |
| usb_pdmon | |

# microcontroller

| name | notes |
| ---- | ----- |
| LPC84x | uart, gpio working |
| CH32X035 | uart, non-isr gpio working |

## drivers

| name | notes |
| ---- | ----- |
| lin | lightweight, callback based API |
| lin_uart | works w/ any uart |
