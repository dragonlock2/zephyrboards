#include "common.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_PROP(TEST_NODE, leds)

#define NUM_LEDS DT_PROP_LEN(TEST_NODE, leds)

static const struct gpio_dt_spec leds[] = {
    DT_FOREACH_PROP_ELEM(TEST_NODE, leds, ELEM_TO_GPIO)
};

static int gpio_test(const struct device *arg) {
    LOG_INF("starting LED test");
    for (int i = 0; i < NUM_LEDS; i++) {
        LOG_INF("flashing LED %d", i);
        gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
        for (int j = 0; j < 6; j++) {
            gpio_pin_toggle_dt(&leds[i]);
            k_msleep(69);
        }
    }
    return 0;
}

SYS_INIT(gpio_test, APPLICATION, 0);

#endif // DT_HAS_PROP(TEST_NODE, leds)
