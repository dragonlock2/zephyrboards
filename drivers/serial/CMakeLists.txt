zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_UART_LPC84X lpc84x_uart.c)

if(CONFIG_UART_LPC84X)
    zephyr_include_directories(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/lpc_miniusart)
    zephyr_sources(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/lpc_miniusart/fsl_usart.c)
endif()
