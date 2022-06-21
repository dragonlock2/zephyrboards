#include "common.h"
#include <drivers/gpio.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(button, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_PROP(TEST_NODE, buttons)

#define NUM_BTNS DT_PROP_LEN(TEST_NODE, buttons)

static const struct gpio_dt_spec btns[] = {
    DT_FOREACH_PROP_ELEM(TEST_NODE, buttons, ELEM_TO_GPIO)
};

static struct gpio_data {
    struct gpio_callback cb_data;
    int aux;
} btns_data[NUM_BTNS];

static void btn_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct gpio_data *data = CONTAINER_OF(cb, struct gpio_data, cb_data);
    LOG_INF("button %d pressed!", data->aux);
}

static int gpio_test(const struct device *arg) {
    LOG_INF("starting button test");
    for (int i = 0; i < NUM_BTNS; i++) {
        LOG_INF("attaching logging interrupt to button %d", i);
        btns_data[i].aux = i;
        gpio_pin_configure_dt(&btns[i], GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&btns[i], GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&btns_data[i].cb_data, btn_cb, BIT(btns[i].pin));
        gpio_add_callback(btns[i].port, &btns_data[i].cb_data);
    }
    return 0;
}

SYS_INIT(gpio_test, APPLICATION, 1);

#endif // DT_NODE_HAS_PROP(TEST_NODE, buttons)
