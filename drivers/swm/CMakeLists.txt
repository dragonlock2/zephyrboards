zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SWM_LPC84X swm_lpc84x.c)

if(CONFIG_SWM_LPC84X)
    set(MCUX_BASE ${ZB_DIR}/../modules/hal/nxp/mcux/)

    zephyr_include_directories(${MCUX_BASE}/mcux-sdk/drivers/swm)
    zephyr_sources(${MCUX_BASE}/mcux-sdk/drivers/swm/fsl_swm.c)
endif()
