#define DT_DRV_COMPAT wch_ch32_uart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

struct ch32_uart_config {
    USART_TypeDef *base;
    uint32_t clock_type, clock_mask;
    uint32_t pin_tx, pin_rx;
    uint32_t remap;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    void (*irq_config_func)(void);
#endif
};

struct ch32_uart_data {
    struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t cb;
    void *user_data;
#endif
};

static int ch32_uart_poll_in(const struct device *dev, unsigned char *c) {
    const struct ch32_uart_config *config = dev->config;
    if (USART_GetFlagStatus(config->base, USART_FLAG_RXNE) == SET) {
        *c = USART_ReceiveData(config->base);
        return 0;
    }
    return -1;
}

static void ch32_uart_poll_out(const struct device *dev, unsigned char c) {
    const struct ch32_uart_config *config = dev->config;
    while (USART_GetFlagStatus(config->base, USART_FLAG_TC) == RESET);
    USART_SendData(config->base, c);
}

static int ch32_uart_err_check(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    int err = 0;

    if (USART_GetITStatus(config->base, USART_IT_ORE_RX) == SET) {
        USART_ClearITPendingBit(config->base, USART_IT_ORE_RX);
        err |= UART_ERROR_OVERRUN;
    }

    if (USART_GetITStatus(config->base, USART_IT_PE) == SET) {
        USART_ClearITPendingBit(config->base, USART_IT_PE);
        err |= UART_ERROR_PARITY;
    }

    if (USART_GetITStatus(config->base, USART_IT_FE) == SET) {
        USART_ClearITPendingBit(config->base, USART_IT_FE);
        err |= UART_ERROR_FRAMING;
    }

    return err;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

static int ch32_uart_configure(const struct device *dev, const struct uart_config *cfg) {
    const struct ch32_uart_config *config = dev->config;
    struct ch32_uart_data *data = dev->data;

    USART_InitTypeDef usart_cfg = {
        .USART_BaudRate = cfg->baudrate,
        .USART_Mode     = USART_Mode_Tx | USART_Mode_Rx,
    };

    switch (cfg->parity) {
        case UART_CFG_PARITY_NONE: usart_cfg.USART_Parity = USART_Parity_No;   break;
        case UART_CFG_PARITY_EVEN: usart_cfg.USART_Parity = USART_Parity_Even; break;
        case UART_CFG_PARITY_ODD:  usart_cfg.USART_Parity = USART_Parity_Odd;  break;
        default: return -ENOTSUP; break;
    }

    switch (cfg->stop_bits) {
        case UART_CFG_STOP_BITS_0_5: usart_cfg.USART_StopBits = USART_StopBits_0_5; break;
        case UART_CFG_STOP_BITS_1:   usart_cfg.USART_StopBits = USART_StopBits_1;   break;
        case UART_CFG_STOP_BITS_1_5: usart_cfg.USART_StopBits = USART_StopBits_1_5; break;
        case UART_CFG_STOP_BITS_2:   usart_cfg.USART_StopBits = USART_StopBits_2;   break;
        default: return -ENOTSUP; break;
    }

    switch (cfg->data_bits) {
        case UART_CFG_DATA_BITS_8: usart_cfg.USART_WordLength = USART_WordLength_8b; break;
        case UART_CFG_DATA_BITS_9: usart_cfg.USART_WordLength = USART_WordLength_9b; break;
        default: return -ENOTSUP; break;
    }

    switch (cfg->flow_ctrl) {
        case UART_CFG_FLOW_CTRL_NONE:    usart_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    break;
        case UART_CFG_FLOW_CTRL_RTS_CTS: usart_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS; break;
        default: return -ENOTSUP; break;
    }

    USART_Init(config->base, &usart_cfg); // note can silently fail
    USART_Cmd(config->base, ENABLE);

    data->uart_cfg = *cfg;
    return 0;
}

static int ch32_uart_config_get(const struct device *dev, struct uart_config *cfg) {
    struct ch32_uart_data *data = dev->data;
    *cfg = data->uart_cfg;
    return 0;
}

#endif // CONFIG_UART_USE_RUNTIME_CONFIGURE

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int ch32_uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len) {
    const struct ch32_uart_config *config = dev->config;
    int num_tx = 0;
    while ((len - num_tx > 0) && (USART_GetFlagStatus(config->base, USART_FLAG_TXE) == SET)) {
        USART_SendData(config->base, tx_data[num_tx++]);
    }
    return num_tx;
}

static int ch32_uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int len) {
    const struct ch32_uart_config *config = dev->config;
    int num_rx = 0;
    while ((len - num_rx > 0) && (USART_GetFlagStatus(config->base, USART_FLAG_RXNE) == SET)) {
        rx_data[num_rx++] = USART_ReceiveData(config->base);
    }
    return num_rx;
}

static void ch32_uart_irq_tx_enable(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    USART_ITConfig(config->base, USART_IT_TXE, ENABLE);
}

static void ch32_uart_irq_tx_disable(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    USART_ITConfig(config->base, USART_IT_TXE, DISABLE);
}

static int ch32_uart_irq_tx_ready(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    return USART_GetITStatus(config->base, USART_IT_TXE) == SET;
}

static void ch32_uart_irq_rx_enable(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    USART_ITConfig(config->base, USART_IT_RXNE, ENABLE);
}

static void ch32_uart_irq_rx_disable(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    USART_ITConfig(config->base, USART_IT_RXNE, DISABLE);
}

static int ch32_uart_irq_tx_complete(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    return USART_GetFlagStatus(config->base, USART_FLAG_TC) == SET;
}

static int ch32_uart_irq_rx_ready(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    return USART_GetITStatus(config->base, USART_IT_RXNE) == SET;
}

static void ch32_uart_irq_err_enable(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    USART_ITConfig(config->base, USART_IT_ORE_RX, ENABLE);
    USART_ITConfig(config->base, USART_IT_PE,     ENABLE);
    USART_ITConfig(config->base, USART_IT_FE,     ENABLE);
}

static void ch32_uart_irq_err_disable(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    USART_ITConfig(config->base, USART_IT_ORE_RX, DISABLE);
    USART_ITConfig(config->base, USART_IT_PE,     DISABLE);
    USART_ITConfig(config->base, USART_IT_FE,     DISABLE);
}

static int ch32_uart_irq_is_pending(const struct device *dev) {
    return ch32_uart_irq_tx_ready(dev) || ch32_uart_irq_rx_ready(dev);
}

static int ch32_uart_irq_update(const struct device *dev) {
    return true; // does nothing
}

static void ch32_uart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data) {
    struct ch32_uart_data *data = dev->data;
    data->cb = cb;
    data->user_data = user_data;
}

static void ch32_uart_isr(const struct device *dev) {
    struct ch32_uart_data *data = dev->data;
    if (data->cb) {
        data->cb(dev, data->user_data);
    }
}

#endif // CONFIG_UART_INTERRUPT_DRIVEN

static int ch32_uart_init(const struct device *dev) {
    const struct ch32_uart_config *config = dev->config;
    struct ch32_uart_data *data = dev->data;
    int err = 0;

    pinctrl_configure_pins(config->pin_tx, GPIO_Mode_AF_PP);
    pinctrl_configure_pins(config->pin_rx, GPIO_Mode_IN_FLOATING);
    pinctrl_apply_state(config->remap);
    clock_control_on(config->clock_type, config->clock_mask);

    err = ch32_uart_configure(dev, &data->uart_cfg);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    config->irq_config_func();
#endif

    return err;
}

static const struct uart_driver_api ch32_uart_driver_api = {
    .poll_in          = ch32_uart_poll_in,
    .poll_out         = ch32_uart_poll_out,
    .err_check        = ch32_uart_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure        = ch32_uart_configure,
    .config_get       = ch32_uart_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill        = ch32_uart_fifo_fill,
    .fifo_read        = ch32_uart_fifo_read,
    .irq_tx_enable    = ch32_uart_irq_tx_enable,
    .irq_tx_disable   = ch32_uart_irq_tx_disable,
    .irq_tx_ready     = ch32_uart_irq_tx_ready,
    .irq_rx_enable    = ch32_uart_irq_rx_enable,
    .irq_rx_disable   = ch32_uart_irq_rx_disable,
    .irq_tx_complete  = ch32_uart_irq_tx_complete,
    .irq_rx_ready     = ch32_uart_irq_rx_ready,
    .irq_err_enable   = ch32_uart_irq_err_enable,
    .irq_err_disable  = ch32_uart_irq_err_disable,
    .irq_is_pending   = ch32_uart_irq_is_pending,
    .irq_update       = ch32_uart_irq_update,
    .irq_callback_set = ch32_uart_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define CH32_UART_IRQ_CONFIG(n)                        \
    static void ch32_uart_##n##_irq_config(void) {     \
        IRQ_CONNECT(DT_INST_PROP_BY_IDX(n, irq, 0), 0, \
            ch32_uart_isr, DEVICE_DT_INST_GET(n), 0);  \
        irq_enable(DT_INST_PROP_BY_IDX(n, irq, 0));                   \
    }
#define CH32_UART_IRQ_INIT(n) .irq_config_func = ch32_uart_##n##_irq_config,
#else
#define CH32_UART_IRQ_CONFIG(n)
#define CH32_UART_IRQ_INIT(n)
#endif // CONFIG_UART_INTERRUPT_DRIVEN

#define CH32_UART_INIT(n)                                             \
    CH32_UART_IRQ_CONFIG(n)                                           \
                                                                      \
    static const struct ch32_uart_config ch32_uart_##n##_config = {   \
        .base       = (USART_TypeDef*) DT_INST_REG_ADDR(n),           \
        .clock_type = DT_INST_PROP_BY_IDX(n, clk, 0),                 \
        .clock_mask = DT_INST_PROP_BY_IDX(n, clk, 1),                 \
        .pin_tx     = DT_INST_PROP_BY_IDX(n, pinctrl, 0),             \
        .pin_rx     = DT_INST_PROP_BY_IDX(n, pinctrl, 1),             \
        .remap      = DT_INST_PROP_BY_IDX(n, remap, 0),               \
        CH32_UART_IRQ_INIT(n)                                         \
    };                                                                \
                                                                      \
    static struct ch32_uart_data ch32_uart_##n##_data = {             \
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
    DEVICE_DT_INST_DEFINE(n,                                          \
        &ch32_uart_init,                                              \
        NULL,                                                         \
        &ch32_uart_##n##_data,                                        \
        &ch32_uart_##n##_config,                                      \
        PRE_KERNEL_1,                                                 \
        CONFIG_SERIAL_INIT_PRIORITY,                                  \
        &ch32_uart_driver_api                                         \
    );

DT_INST_FOREACH_STATUS_OKAY(CH32_UART_INIT)
