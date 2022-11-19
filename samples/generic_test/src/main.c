#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "common.h"

const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main() {
#if DT_PROP(TEST_NODE, sd)
    extern int sd_test(void);
    sd_test(); // can't use SYS_INIT :(
#endif // DT_PROP(TEST_NODE, sd)

    printk("Flashing led0 in main at 1Hz so it does something...\r\n");
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(500);
    }
}