#define DT_DRV_COMPAT nxp_lpc84x_uart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
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
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    void (*irq_config_func)(const struct device *dev);
#endif
};

struct lpc84x_uart_data {
    struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t cb;
    void *user_data;
#endif
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

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

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

    uint32_t enabled_irq = USART_GetEnabledInterrupts(config->base);
    if (USART_Init(config->base, &usart_config, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) != kStatus_Success) {
        return -EINVAL;
    }
    USART_EnableInterrupts(config->base, enabled_irq);

    data->uart_cfg = *cfg;
    return 0;
}

static int lpc84x_uart_config_get(const struct device *dev, struct uart_config *cfg) {
    struct lpc84x_uart_data *data = dev->data;
    *cfg = data->uart_cfg;
    return 0;
}

#endif // CONFIG_UART_USE_RUNTIME_CONFIGURE

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int lpc84x_uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len) {
    const struct lpc84x_uart_config *config = dev->config;
    int num_tx = 0;
    while ((len - num_tx > 0) && (USART_GetStatusFlags(config->base) & kUSART_TxReady)) {
        USART_WriteByte(config->base, tx_data[num_tx++]);
    }
    return num_tx;
}

static int lpc84x_uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int len) {
    const struct lpc84x_uart_config *config = dev->config;
    int num_rx = 0;
    while ((len - num_rx > 0) && (USART_GetStatusFlags(config->base) & kUSART_RxReady)) {
        rx_data[num_rx++] = USART_ReadByte(config->base);
    }
    return num_rx;
}

static void lpc84x_uart_irq_tx_enable(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    USART_EnableInterrupts(config->base, kUSART_TxReadyInterruptEnable);
}

static void lpc84x_uart_irq_tx_disable(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    USART_DisableInterrupts(config->base, kUSART_TxReadyInterruptEnable);
}

static int lpc84x_uart_irq_tx_ready(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    return (USART_GetEnabledInterrupts(config->base) & kUSART_TxReadyInterruptEnable) &&
        (USART_GetStatusFlags(config->base) & kUSART_TxReady);
}

static void lpc84x_uart_irq_rx_enable(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    USART_EnableInterrupts(config->base, kUSART_RxReadyInterruptEnable);
}

static void lpc84x_uart_irq_rx_disable(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    USART_DisableInterrupts(config->base, kUSART_RxReadyInterruptEnable);
}

static int lpc84x_uart_irq_tx_complete(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    return (USART_GetStatusFlags(config->base) & kUSART_TxIdleFlag) != 0;
}

static int lpc84x_uart_irq_rx_ready(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    return (USART_GetEnabledInterrupts(config->base) & kUSART_RxReadyInterruptEnable) &&
        (USART_GetStatusFlags(config->base) & kUSART_RxReady);
}

static void lpc84x_uart_irq_err_enable(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    USART_EnableInterrupts(config->base, kUSART_HardwareOverRunInterruptEnable |
        kUSART_ParityErrorInterruptEnable | kUSART_FramErrorInterruptEnable);
}

static void lpc84x_uart_irq_err_disable(const struct device *dev) {
    const struct lpc84x_uart_config *config = dev->config;
    USART_DisableInterrupts(config->base, kUSART_HardwareOverRunInterruptEnable |
        kUSART_ParityErrorInterruptEnable | kUSART_FramErrorInterruptEnable);
}

static int lpc84x_uart_irq_is_pending(const struct device *dev) {
    return lpc84x_uart_irq_tx_ready(dev) || lpc84x_uart_irq_rx_ready(dev);
}

static int lpc84x_uart_irq_update(const struct device *dev) {
    return true; // does nothing
}

static void lpc84x_uart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data) {
    struct lpc84x_uart_data *data = dev->data;
    data->cb = cb;
    data->user_data = user_data;
}

static void lpc84x_uart_isr(const struct device *dev) {
    struct lpc84x_uart_data *data = dev->data;
    if (data->cb) {
        data->cb(dev, data->user_data);
    }
}

#endif // CONFIG_UART_INTERRUPT_DRIVEN

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

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    config->irq_config_func(dev);
#endif

    return err;
}

static const struct uart_driver_api lpc84x_uart_driver_api = {
    .poll_in          = lpc84x_uart_poll_in,
    .poll_out         = lpc84x_uart_poll_out,
    .err_check        = lpc84x_uart_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure        = lpc84x_uart_configure,
    .config_get       = lpc84x_uart_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill        = lpc84x_uart_fifo_fill,
    .fifo_read        = lpc84x_uart_fifo_read,
    .irq_tx_enable    = lpc84x_uart_irq_tx_enable,
    .irq_tx_disable   = lpc84x_uart_irq_tx_disable,
    .irq_tx_ready     = lpc84x_uart_irq_tx_ready,
    .irq_rx_enable    = lpc84x_uart_irq_rx_enable,
    .irq_rx_disable   = lpc84x_uart_irq_rx_disable,
    .irq_tx_complete  = lpc84x_uart_irq_tx_complete,
    .irq_rx_ready     = lpc84x_uart_irq_rx_ready,
    .irq_err_enable   = lpc84x_uart_irq_err_enable,
    .irq_err_disable  = lpc84x_uart_irq_err_disable,
    .irq_is_pending   = lpc84x_uart_irq_is_pending,
    .irq_update       = lpc84x_uart_irq_update,
    .irq_callback_set = lpc84x_uart_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define LPC84X_UART_IRQ_CONFIG(n)                                        \
    static void lpc84x_uart_##n##_irq_config(const struct device *dev) { \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),           \
            lpc84x_uart_isr, DEVICE_DT_INST_GET(n), 0);                  \
        irq_enable(DT_INST_IRQN(n));                                     \
    }
#define LPC84X_UART_IRQ_INIT(n) .irq_config_func = lpc84x_uart_##n##_irq_config,
#else
#define LPC84X_UART_IRQ_CONFIG(n)
#define LPC84X_UART_IRQ_INIT(n)
#endif // CONFIG_UART_INTERRUPT_DRIVEN

#define LPC84X_UART_INIT(n)                                             \
    LPC84X_UART_IRQ_CONFIG(n)                                           \
                                                                        \
    static const struct lpc84x_uart_config lpc84x_uart_##n##_config = { \
        .base   = (USART_Type*) DT_INST_REG_ADDR(n),                    \
        .clock  = DT_INST_PROP_BY_IDX(n, clk, 0),                       \
        .swm_tx = DT_INST_PROP_BY_IDX(n, swm, 0),                       \
        .swm_rx = DT_INST_PROP_BY_IDX(n, swm, 1),                       \
        .pin_tx = DT_INST_PROP_BY_IDX(n, pinctrl, 0),                   \
        .pin_rx = DT_INST_PROP_BY_IDX(n, pinctrl, 1),                   \
        LPC84X_UART_IRQ_INIT(n)                                         \
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
