if(CONFIG_PINMUX_MCUX)
  zephyr_library()
  zephyr_library_sources(pinmux.c)
endif()
