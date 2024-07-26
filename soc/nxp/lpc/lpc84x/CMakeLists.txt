zephyr_include_directories(
    .
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/swm
)

zephyr_sources(
    soc.c
    ${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk/drivers/swm/fsl_swm.c
)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
