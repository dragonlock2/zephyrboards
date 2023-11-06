zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_GPIO_LPC84X gpio_lpc84x.c)

if(CONFIG_GPIO_LPC84X)
    zephyr_include_directories(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/lpc_gpio)
    zephyr_include_directories(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/pint)
    zephyr_sources(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/lpc_gpio/fsl_gpio.c)
    zephyr_sources(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/pint/fsl_pint.c)
endif()
