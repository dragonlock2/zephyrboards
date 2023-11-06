#define DT_DRV_COMPAT nxp_lpc84x_uart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <fsl_iocon.h>
#include <fsl_swm.h>
#include <fsl_usart.h>

/* inspired by nxp,kinetis-uart in uart_mcux.c */

#define PIN_GET_SWM(x)    (((x) >> 8) & 0xFF)
#define PIN_GET_IOCON(x)  ((x) & 0xFF)

struct lpc84x_uart_config {
    USART_Type *base;
    clock_ip_name_t clock;
    swm_select_movable_t swm_tx, swm_rx;
    uint32_t pin_tx, pin_rx;
};

struct lpc84x_uart_data {
    struct uart_config uart_cfg;
};

static int lpc84x_uart_poll_in(const struct device *dev, unsigned char *c) {
    const struct lpc84x_uart_config *config = dev->config;
    if (USART_GetStatusFlags(config->base) & kUSART_RxReady) {
        *c = USART_ReadByte(config->base);
        return 0;
    }
    return -1;
}

static void lpc84x_uart_poll_out(const struct device *dev, unsigned char c) {
    const struct lpc84x_uart_config *config = dev->config;
    while (!(USART_GetStatusFlags(config->base) & kUSART_TxReady));
    USART_WriteByte(config->base, c);
}

static int lpc84x_uart_err_check(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    uint32_t flags = USART_GetStatusFlags(config->base);
    int err = 0;

    if (flags & kUSART_HardwareOverrunFlag) {
        err |= UART_ERROR_OVERRUN;
    }

    if (flags & kUSART_ParityErrorFlag) {
        err |= UART_ERROR_PARITY;
    }

    if (flags & kUSART_FramErrorFlag) {
        err |= UART_ERROR_FRAMING;
    }

    USART_ClearStatusFlags(config->base, kUSART_HardwareOverrunFlag | kUSART_ParityErrorFlag | kUSART_FramErrorFlag);
    return err;
}

static int lpc84x_uart_configure(const struct device *dev, const struct uart_config *cfg) {
    const struct lpc84x_uart_config *config = dev->config;
    struct lpc84x_uart_data *data = dev->data;

    usart_config_t usart_config;
    USART_GetDefaultConfig(&usart_config);

    usart_config.enableTx     = true;
    usart_config.enableRx     = true;
    usart_config.baudRate_Bps = cfg->baudrate;

    switch (cfg->stop_bits) {
        case UART_CFG_STOP_BITS_1: usart_config.stopBitCount = kUSART_OneStopBit; break;
        case UART_CFG_STOP_BITS_2: usart_config.stopBitCount = kUSART_TwoStopBit; break;
        default: return -ENOTSUP; break;
    }

    switch (cfg->flow_ctrl) {
        case UART_CFG_FLOW_CTRL_NONE:    usart_config.enableHardwareFlowControl = false; break;
        case UART_CFG_FLOW_CTRL_RTS_CTS: usart_config.enableHardwareFlowControl = true;  break;
        default: return -ENOTSUP; break;
    }

    switch (cfg->parity) {
        case UART_CFG_PARITY_NONE: usart_config.parityMode = kUSART_ParityDisabled; break;
        case UART_CFG_PARITY_EVEN: usart_config.parityMode = kUSART_ParityEven;     break;
        case UART_CFG_PARITY_ODD:  usart_config.parityMode = kUSART_ParityOdd;      break;
        default: return -ENOTSUP; break;
    }

    if (USART_Init(config->base, &usart_config, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) != kStatus_Success) {
        return -EINVAL;
    }

    data->uart_cfg = *cfg;
    return 0;
}

static int lpc84x_uart_config_get(const struct device *dev, struct uart_config *cfg) {
    struct lpc84x_uart_data *data = dev->data;
    *cfg = data->uart_cfg;
    return 0;
}

static int lpc84x_uart_init(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    struct lpc84x_uart_data *data = dev->data;
    int err = 0;

    SWM_SetMovablePinSelect(SWM0, config->swm_tx, PIN_GET_SWM(config->pin_tx));
    SWM_SetMovablePinSelect(SWM0, config->swm_rx, PIN_GET_SWM(config->pin_rx));

    IOCON_PinMuxSet(IOCON, PIN_GET_IOCON(config->pin_tx), IOCON_HYS_EN);
    IOCON_PinMuxSet(IOCON, PIN_GET_IOCON(config->pin_rx), IOCON_HYS_EN);

    CLOCK_EnableClock(config->clock);

    err = lpc84x_uart_configure(dev, &data->uart_cfg);

    return err;
}

static const struct uart_driver_api lpc84x_uart_driver_api = {
    .poll_in    = lpc84x_uart_poll_in,
    .poll_out   = lpc84x_uart_poll_out,
    .err_check  = lpc84x_uart_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure  = lpc84x_uart_configure,
    .config_get = lpc84x_uart_config_get,
#endif
    // TODO interrupt support
};

#define LPC84X_UART_INIT(n)                                             \
    static const struct lpc84x_uart_config lpc84x_uart_##n##_config = { \
        .base   = (USART_Type*) DT_INST_REG_ADDR(n),                    \
        .clock  = DT_INST_PROP_BY_IDX(n, clk, 0),                       \
        .swm_tx = DT_INST_PROP_BY_IDX(n, swm, 0),                       \
        .swm_rx = DT_INST_PROP_BY_IDX(n, swm, 1),                       \
        .pin_tx = DT_INST_PROP_BY_IDX(n, pinctrl, 0),                   \
        .pin_rx = DT_INST_PROP_BY_IDX(n, pinctrl, 1),                   \
    };                                                                  \
                                                                        \
    static struct lpc84x_uart_data lpc84x_uart_##n##_data = {           \
        .uart_cfg = {                                                   \
            .stop_bits = UART_CFG_STOP_BITS_1,                          \
            .data_bits = UART_CFG_DATA_BITS_8,                          \
            .baudrate  = DT_INST_PROP(n, current_speed),                \
            .parity    = UART_CFG_PARITY_NONE,                          \
            .flow_ctrl = DT_INST_PROP(n, hw_flow_control) ?             \
                UART_CFG_FLOW_CTRL_RTS_CTS : UART_CFG_FLOW_CTRL_NONE,   \
        },                                                              \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(n,                                            \
        &lpc84x_uart_init,                                              \
        NULL,                                                           \
        &lpc84x_uart_##n##_data,                                        \
        &lpc84x_uart_##n##_config,                                      \
        PRE_KERNEL_1,                                                   \
        CONFIG_SERIAL_INIT_PRIORITY,                                    \
        &lpc84x_uart_driver_api                                         \
    );

DT_INST_FOREACH_STATUS_OKAY(LPC84X_UART_INIT)
