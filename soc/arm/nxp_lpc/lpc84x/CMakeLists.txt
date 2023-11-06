zephyr_library()

zephyr_library_sources(
    soc.c
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/swm/fsl_swm.c
)

zephyr_include_directories( # add to global includes
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/swm
)
