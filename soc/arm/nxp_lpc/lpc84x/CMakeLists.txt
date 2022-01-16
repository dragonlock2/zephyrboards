zephyr_library()

zephyr_library_sources(soc.c)

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
  )

# include mcux-sdk
set(MCUX_BASE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../ext/mcux-sdk/)
string(TOUPPER ${CONFIG_SOC} MCUX_DEVICE)
set(MCUX_CPU CPU_${CONFIG_SOC_PART_NUMBER})

zephyr_include_directories(${MCUX_BASE}/devices/${MCUX_DEVICE})
zephyr_include_directories(${MCUX_BASE}/devices/${MCUX_DEVICE}/drivers/)
FILE(GLOB mcux_drivers ${MCUX_BASE}/drivers/*)
zephyr_include_directories(${mcux_drivers})

zephyr_compile_definitions(${MCUX_CPU})

zephyr_sources(${MCUX_BASE}/devices/${MCUX_DEVICE}/drivers/fsl_clock.c)
zephyr_sources(${MCUX_BASE}/devices/${MCUX_DEVICE}/drivers/fsl_power.c)
zephyr_sources(${MCUX_BASE}/devices/${MCUX_DEVICE}/drivers/fsl_reset.c)
