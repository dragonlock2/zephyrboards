#define DT_DRV_COMPAT zephyrboards_lin_uart

#include <zephyrboards/drivers/lin.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lin_uart, CONFIG_LIN_LOG_LEVEL);

struct lin_uart_config {
    const struct device *uart_dev;
    const uint32_t max_wait_percent;
};

struct lin_uart_data {
    uint32_t bitrate;
    // TODO the filters, precomputed delay times, timers, mode, states
};

static int lin_uart_set_mode(const struct device *dev, enum lin_mode mode) {
    return -ENOTSUP;
}

static int lin_uart_set_bitrate(const struct device *dev, uint32_t bitrate) {
    return -ENOTSUP;
}

static int lin_uart_send(const struct device *dev, const struct zlin_frame *frame,
                         k_timeout_t timeout, lin_tx_callback_t callback, void *user_data) {
    return -ENOTSUP;
}

static int lin_uart_receive(const struct device *dev, k_timeout_t timeout) {
    return -ENOTSUP;
}

static int lin_uart_add_rx_filter(const struct device *dev, lin_rx_callback_t callback,
                                  void *user_data, const struct zlin_filter *filter) {
    return -ENOTSUP;
}

static void lin_uart_remove_rx_filter(const struct device *dev, int filter_id) {

}

static void lin_uart_set_state_change_callback(const struct device *dev,
                                               lin_state_change_callback_t callback,
                                               void *user_data) {

}

static int lin_uart_init(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;

    if (!device_is_ready(cfg->uart_dev)) {
        return -ENODEV;
    }
    return 0;
}

static struct lin_driver_api lin_uart_api = {
    .set_mode = lin_uart_set_mode,
    .set_bitrate = lin_uart_set_bitrate,
    .send = lin_uart_send,
    .receive = lin_uart_receive,
    .add_rx_filter = lin_uart_add_rx_filter,
    .remove_rx_filter = lin_uart_remove_rx_filter,
    .set_state_change_callback = lin_uart_set_state_change_callback,
};

#define LIN_UART_INIT(n)                                       \
    const struct lin_uart_config lin_uart_config_##n = {       \
        .uart_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),          \
        .max_wait_percent = DT_INST_PROP(n, max_wait_percent), \
    };                                                         \
                                                               \
    struct lin_uart_data lin_uart_data_##n = {                 \
        .bitrate = DT_INST_PROP(n, bitrate),                   \
    };                                                         \
                                                               \
    DEVICE_DT_INST_DEFINE(n,                                   \
                          &lin_uart_init,                      \
                          NULL,                                \
                          &lin_uart_data_##n,                  \
                          &lin_uart_config_##n,                \
                          POST_KERNEL,                         \
                          CONFIG_LIN_INIT_PRIORITY,            \
                          &lin_uart_api);

DT_INST_FOREACH_STATUS_OKAY(LIN_UART_INIT)
