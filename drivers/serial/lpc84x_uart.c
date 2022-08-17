#define DT_DRV_COMPAT nxp_lpc84x_uart

/* Inspired by nxp,kinetis-uart */

#include <errno.h>
#include <device.h>

#include <drivers/uart.h>
#include <drivers/clock_control.h>
#include <zephyrboards/drivers/swm.h>

#include <fsl_usart.h>
#include <fsl_iocon.h>

struct lpc84x_uart_config {
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

struct lpc84x_uart_data {
    struct uart_config uart_cfg;
};

static int lpc84x_uart_configure(const struct device *dev,
                   const struct uart_config *cfg) {
    const struct lpc84x_uart_config *config = dev->config;
    struct lpc84x_uart_data *data = dev->data;

    usart_config_t usart_config;
    USART_GetDefaultConfig(&usart_config);

    usart_config.enableTx = true;
    usart_config.enableRx = true;
    usart_config.baudRate_Bps = cfg->baudrate;

    switch (cfg->stop_bits) {
        case UART_CFG_STOP_BITS_1:
            usart_config.stopBitCount = kUSART_OneStopBit;
            break;
        case UART_CFG_STOP_BITS_2:
            usart_config.stopBitCount = kUSART_TwoStopBit;
            break;
        default:
            return -ENOTSUP;
    }

    switch (cfg->flow_ctrl) {
        case UART_CFG_FLOW_CTRL_NONE:
            usart_config.enableHardwareFlowControl = false;
            break;
        case UART_CFG_FLOW_CTRL_RTS_CTS:
            usart_config.enableHardwareFlowControl = true;
            break;
        default:
            return -ENOTSUP;
    }

    switch (cfg->parity) {
        case UART_CFG_PARITY_NONE:
            usart_config.parityMode = kUSART_ParityDisabled;
            break;
        case UART_CFG_PARITY_EVEN:
            usart_config.parityMode = kUSART_ParityEven;
            break;
        case UART_CFG_PARITY_ODD:
            usart_config.parityMode = kUSART_ParityOdd;
            break;
        default:
            return -ENOTSUP;
    }

    status_t retval = USART_Init(config->base, &usart_config,
        CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
    if (retval != kStatus_Success) {
        return -EINVAL;
    }

    data->uart_cfg = *cfg;

    return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int lpc84x_uart_config_get(const struct device *dev,
                struct uart_config *cfg) {
    struct lpc84x_uart_data *data = dev->data;
    *cfg = data->uart_cfg;
    return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int lpc84x_uart_poll_in(const struct device *dev, unsigned char *c) {
    const struct lpc84x_uart_config *config = dev->config;
    uint32_t flags = USART_GetStatusFlags(config->base);

    if (flags & USART_STAT_RXRDY_MASK) {
        *c = USART_ReadByte(config->base);
        return 0;
    }

    return -1;
}

static void lpc84x_uart_poll_out(const struct device *dev, unsigned char c) {
    const struct lpc84x_uart_config *config = dev->config;

    while (!(USART_GetStatusFlags(config->base) & USART_STAT_TXRDY_MASK));

    USART_WriteByte(config->base, c);
}

static int lpc84x_uart_err_check(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;

    uint32_t flags = USART_GetStatusFlags(config->base);
    int err = 0;

    if (flags & USART_STAT_OVERRUNINT_MASK) {
        err |= UART_ERROR_OVERRUN;
    }

    if (flags & USART_STAT_PARITYERRINT_MASK) {
        err |= UART_ERROR_PARITY;
    }

    if (flags & USART_STAT_FRAMERRINT_MASK) {
        err |= UART_ERROR_FRAMING;
    }

    USART_ClearStatusFlags(config->base, USART_STAT_OVERRUNINT_MASK |
        USART_STAT_PARITYERRINT_MASK | USART_STAT_FRAMERRINT_MASK);

    return err;
}

static int lpc84x_uart_init(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    struct lpc84x_uart_data *data = dev->data;
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

    err = lpc84x_uart_configure(dev, &data->uart_cfg);
    if (err != 0) {
        return err;
    }

    return 0;
}

static const struct uart_driver_api lpc84x_uart_driver_api = {
    .poll_in = lpc84x_uart_poll_in,
    .poll_out = lpc84x_uart_poll_out,
    .err_check = lpc84x_uart_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = lpc84x_uart_configure,
    .config_get = lpc84x_uart_config_get,
#endif
};

#define LPC84X_UART_DECLARE_CFG(n)                                        \
static const struct lpc84x_uart_config lpc84x_uart_##n##_config = {       \
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

#define LPC84X_UART_INIT(n)                                           \
                                                                      \
    static struct lpc84x_uart_data lpc84x_uart_##n##_data = {         \
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
    static const struct lpc84x_uart_config lpc84x_uart_##n##_config;  \
                                                                      \
    DEVICE_DT_INST_DEFINE(n,                                          \
                &lpc84x_uart_init,                                    \
                NULL,                                                 \
                &lpc84x_uart_##n##_data,                              \
                &lpc84x_uart_##n##_config,                            \
                PRE_KERNEL_1,                                         \
                CONFIG_SERIAL_INIT_PRIORITY,                          \
                &lpc84x_uart_driver_api);                             \
                                                                      \
    LPC84X_UART_DECLARE_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_UART_INIT)
