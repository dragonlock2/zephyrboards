#define DT_DRV_COMPAT nxp_lpc84x_uart

/* Inspired by nxp,kinetis-uart */

#include <errno.h>
#include <device.h>

#include <drivers/uart.h>
#include <drivers/clock_control.h>
#include <drivers/swm.h>

#include <fsl_usart.h>
#include <fsl_iocon.h>

struct uart_lpc84x_config {
    USART_Type *base;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    uint8_t iocon_tx;
    uint8_t iocon_rx;
    const struct device *swm_tx;
    swm_pin_t swm_pin_tx;
    swm_function_t swm_func_tx;
    const struct device *swm_rx;
    swm_pin_t swm_pin_rx;
    swm_function_t swm_func_rx;
};

struct uart_lpc84x_data {
    struct uart_config uart_cfg;
};

static int uart_lpc84x_configure(const struct device *dev,
                   const struct uart_config *cfg) {
    const struct uart_lpc84x_config *config = dev->config;
    struct uart_lpc84x_data *data = dev->data;



    return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_lpc84x_config_get(const struct device *dev,
                struct uart_config *cfg) {
    struct uart_lpc84x_data *data = dev->data;
    *cfg = data->uart_cfg;
    return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_lpc84x_poll_in(const struct device *dev, unsigned char *c) {
    const struct uart_lpc84x_config *config = dev->config;
    return 0;
}

static void uart_lpc84x_poll_out(const struct device *dev, unsigned char c) {
    const struct uart_lpc84x_config *config = dev->config;
}

static int uart_lpc84x_err_check(const struct device *dev)
{
    const struct uart_lpc84x_config *config = dev->config;
    return 0;
}

static int uart_lpc84x_init(const struct device *dev)
{
    const struct uart_lpc84x_config *config = dev->config;
    struct uart_lpc84x_data *data = dev->data;
    int err;

    // enable clock
    clock_control_on(config->clock_dev, config->clock_subsys);

    // configure IOCON
    const uint32_t DEBUG_UART = IOCON_MODE_PULLUP  | IOCON_HYS_EN;
    IOCON_PinMuxSet(IOCON, config->iocon_tx, DEBUG_UART);
    IOCON_PinMuxSet(IOCON, config->iocon_rx, DEBUG_UART);

    // configure SWM
    swm_assign(config->swm_tx, config->swm_pin_tx, config->swm_func_tx);
    swm_assign(config->swm_rx, config->swm_pin_rx, config->swm_func_rx);

    err = uart_lpc84x_configure(dev, &data->uart_cfg);
    if (err != 0) {
        return err;
    }

    return 0;
}

static const struct uart_driver_api uart_lpc84x_driver_api = {
    .poll_in = uart_lpc84x_poll_in,
    .poll_out = uart_lpc84x_poll_out,
    .err_check = uart_lpc84x_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = uart_lpc84x_configure,
    .config_get = uart_lpc84x_config_get,
#endif
};

#define UART_LPC84X_DECLARE_CFG(n)                                        \
static const struct uart_lpc84x_config uart_lpc84x_##n##_config = {       \
    .base = (USART_Type *)DT_INST_REG_ADDR(n),                            \
    .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                   \
    .clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name), \
    .iocon_tx = DT_INST_PROP_BY_IDX(n, iocon_pins, 0),                    \
    .iocon_rx = DT_INST_PROP_BY_IDX(n, iocon_pins, 1),                    \
    .swm_tx = DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(n, swms, 0)),          \
    .swm_pin_tx = (swm_pin_t)DT_INST_PROP_BY_IDX(n, swm_pins, 0),         \
    .swm_func_tx = (swm_function_t)DT_INST_PROP_BY_IDX(n, swmfuncs, 0),   \
    .swm_rx = DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(n, swms, 1)),          \
    .swm_pin_rx = (swm_pin_t)DT_INST_PROP_BY_IDX(n, swm_pins, 1),         \
    .swm_func_rx = (swm_function_t)DT_INST_PROP_BY_IDX(n, swmfuncs, 1),   \
}

#define UART_LPC84X_INIT(n)                                           \
                                                                      \
    static struct uart_lpc84x_data uart_lpc84x_##n##_data = {         \
        .uart_cfg = {                                                 \
            .stop_bits = UART_CFG_STOP_BITS_1,                        \
            .data_bits = UART_CFG_DATA_BITS_8,                        \
            .baudrate  = DT_INST_PROP(n, current_speed),              \
            .parity    = UART_CFG_PARITY_NONE,                        \
            .flow_ctrl = DT_INST_PROP(n, hw_flow_control) ?           \
                UART_CFG_FLOW_CTRL_RTS_CTS : UART_CFG_FLOW_CTRL_NONE, \
        },                                                            \
    };                                                                \
                                                                      \
    static const struct uart_lpc84x_config uart_lpc84x_##n##_config;  \
                                                                      \
    DEVICE_DT_INST_DEFINE(n,                                          \
                &uart_lpc84x_init,                                    \
                NULL,                                                 \
                &uart_lpc84x_##n##_data,                              \
                &uart_lpc84x_##n##_config,                            \
                PRE_KERNEL_1,                                         \
                CONFIG_SERIAL_INIT_PRIORITY,                          \
                &uart_lpc84x_driver_api);                             \
                                                                      \
    UART_LPC84X_DECLARE_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(UART_LPC84X_INIT)