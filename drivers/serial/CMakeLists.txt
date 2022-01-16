zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_UART_LPC84X lpc84x_uart.c)

if(CONFIG_UART_LPC84X)
    set(MCUX_BASE ${ZB_DIR}/../modules/hal/nxp/mcux/)

    zephyr_include_directories(${MCUX_BASE}/mcux-sdk/drivers/lpc_miniusart)
    zephyr_sources(${MCUX_BASE}/mcux-sdk/drivers/lpc_miniusart/fsl_usart.c)
endif()
