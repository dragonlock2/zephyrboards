zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_MCUX_SDHC mcux_sdhc.c)

if(CONFIG_MCUX_SDHC)
    zephyr_include_directories(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/sdhc)
    zephyr_sources(${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/sdhc/fsl_sdhc.c)
endif()
