cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(generic_test)

target_sources(app PRIVATE 
    src/main.c
    src/led_test.c
    src/button_test.c
    src/spi_test.c
    src/i2c_test.c
    src/uart_test.c
    src/usb_test.c
    src/adc_test.c
    src/dac_test.c
    src/sd_test.c
    src/can_test.c
    src/network_test.c
)
