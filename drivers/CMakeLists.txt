add_subdirectory(usb)
add_subdirectory(usb_c)

add_subdirectory_ifdef(CONFIG_ADC    adc)
add_subdirectory_ifdef(CONFIG_GPIO   gpio)
add_subdirectory_ifdef(CONFIG_LIN    lin)
add_subdirectory_ifdef(CONFIG_SDHC   sdhc)
add_subdirectory_ifdef(CONFIG_SERIAL serial)
