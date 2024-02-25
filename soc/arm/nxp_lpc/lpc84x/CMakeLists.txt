zephyr_library()

zephyr_library_sources(
    soc.c
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/swm/fsl_swm.c
)

zephyr_include_directories( # add to global includes
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/swm
)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
