#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>

static int usb_pdmon_init(void) {
#ifdef CONFIG_USBC_VBUS_DRIVER
    // stack never calls usbc_vbus_enable...
    const struct gpio_dt_spec vbus_en = GPIO_DT_SPEC_GET(DT_NODELABEL(vbus_en), gpios);
    gpio_pin_configure_dt(&vbus_en, GPIO_OUTPUT_ACTIVE);
#endif
    return 0;
}

SYS_INIT(usb_pdmon_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY); // after usbc_vbus_adc.c
