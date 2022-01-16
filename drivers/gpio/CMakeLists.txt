zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_GPIO_LPC84X gpio_lpc84x.c)

# need to manually include bc shared with nxp,lpc-gpio
if(CONFIG_GPIO_LPC84X)
    set(MCUX_BASE ${ZB_DIR}/../modules/hal/nxp/mcux/)

    zephyr_include_directories(${MCUX_BASE}/mcux-sdk/drivers/lpc_gpio)
    zephyr_sources(${MCUX_BASE}/mcux-sdk/drivers/lpc_gpio/fsl_gpio.c)

    zephyr_include_directories(${MCUX_BASE}/mcux-sdk/drivers/pint)
    zephyr_sources(${MCUX_BASE}/mcux-sdk/drivers/pint/fsl_pint.c)
endif()
