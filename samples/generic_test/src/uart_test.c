#include "common.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_PROP(TEST_NODE, uart)

#define NUM_UART DT_PROP_LEN(TEST_NODE, uart)

static const struct device *uarts[] = {
    DT_FOREACH_PROP_ELEM(TEST_NODE, uart, ELEM_TO_DEVICE)
};

static int uart_test(void) {
    LOG_INF("starting UART test");
    char c;
    for (int i = 0; i < NUM_UART; i++) {
        LOG_INF("sent 'a' on UART %d", i);
        uart_poll_out(uarts[i], 'a');
        k_msleep(10);
        if (uart_poll_in(uarts[i], &c) < 0) {
            LOG_ERR("didn't receive char in time :(");
        } else {
            LOG_INF("received '%c'", c);
        }
    }
    return 0;
}

SYS_INIT(uart_test, APPLICATION, 4);

#endif // DT_NODE_HAS_PROP(TEST_NODE, uart)
