#define DT_DRV_COMPAT virtual_lin_uart

#include <zephyrboards/drivers/lin.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lin_uart, CONFIG_LIN_LOG_LEVEL);

/* private constants */
// from LIN 2.2A Table 2.4: Valid frame identifiers
static const uint8_t PID[LIN_NUM_ID] = {
    0x80, 0xc1, 0x42, 0x03, 0xc4, 0x85, 0x06, 0x47,
    0x08, 0x49, 0xca, 0x8b, 0x4c, 0x0d, 0x8e, 0xcf,
    0x50, 0x11, 0x92, 0xd3, 0x14, 0x55, 0xd6, 0x97,
    0xd8, 0x99, 0x1a, 0x5b, 0x9c, 0xdd, 0x5e, 0x1f,
    0x20, 0x61, 0xe2, 0xa3, 0x64, 0x25, 0xa6, 0xe7,
    0xa8, 0xe9, 0x6a, 0x2b, 0xec, 0xad, 0x2e, 0x6f,
    0xf0, 0xb1, 0x32, 0x73, 0xb4, 0xf5, 0x76, 0x37,
    0x78, 0x39, 0xba, 0xfb, 0x3c, 0x7d, 0xfe, 0xbf,
};

static const uint8_t BREAK = 0x00;
static const uint8_t SYNC  = 0x55;

/* private types */
enum lin_uart_state {
    LIN_UART_STATE_IDLE,
    LIN_UART_STATE_BREAK,
    LIN_UART_STATE_SYNC,
    LIN_UART_STATE_PID,
    LIN_UART_STATE_SEND_RESPONSE,
    LIN_UART_STATE_SEND_CHECKSUM,
    LIN_UART_STATE_RECV_RESPONSE,
};

struct lin_uart_config {
    const struct device *uart_dev;
    uint32_t max_wait_percent;
};

struct lin_uart_data {
    struct k_sem lock;
    bool irq_has_lock;
    struct k_sem irq_lock;

    uint32_t bitrate;
    uint32_t break_bitrate;
    struct uart_config uart_cfg;
    struct {
        lin_header_callback_t header;
        void *header_data;
        lin_tx_callback_t tx;
        void *tx_data;
        lin_rx_callback_t rx;
        void *rx_data;
    } callbacks;

    enum lin_mode mode;
    enum lin_uart_state state;
    bool sending;
    struct lin_frame frame;
    size_t frame_idx;
    uint8_t checksum; 

    const struct device *lin_dev;
    struct k_timer timer;
    k_timeout_t timeout;
    k_timeout_t break_timeout;
    k_timeout_t sync_timeout;
};

/* private helpers */
static void lin_uart_send_break(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    data->uart_cfg.baudrate = data->break_bitrate;
    if (uart_configure(cfg->uart_dev, &data->uart_cfg)) {
        LOG_ERR("unable to config for a break");
    }
    data->state = LIN_UART_STATE_BREAK;
    k_timer_start(&data->timer, data->break_timeout, K_FOREVER);
    uart_poll_out(cfg->uart_dev, BREAK);
}

static void lin_uart_send_byte(const struct device *dev, uint8_t c) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    k_timer_start(&data->timer, data->timeout, K_FOREVER);
    uart_poll_out(cfg->uart_dev, c);
}

/* NOTE: On some UART after a baudrate change, there's a latency. On STM32 and NXP,
 * it outputs one idle char at the new baudrate which puts us just within the 40%
 * margin LIN specifies.
 */
static void lin_uart_set_nominal_bitrate(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    data->uart_cfg.baudrate = data->bitrate;
    if (uart_configure(cfg->uart_dev, &data->uart_cfg)) {
        LOG_ERR("unable to config for nominal bitrate");
    }
}

static void lin_uart_clear_isr(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    while (uart_irq_update(cfg->uart_dev) && uart_irq_is_pending(cfg->uart_dev)) {
        if (uart_irq_rx_ready(cfg->uart_dev)) {
            uint8_t dummy;
            uart_err_check(cfg->uart_dev);
            uart_fifo_read(cfg->uart_dev, &dummy, 1);
        }
    }
}

static void lin_uart_release_isr(struct lin_uart_data *data) {
    data->irq_has_lock = false;
    k_sem_give(&data->lock);
}

static void lin_uart_update_checksum(struct lin_uart_data *data, uint8_t c) {
    uint16_t sum = data->checksum + c;
    if (sum >= 256) {
        sum -= 255;
    }
    data->checksum = sum;
}

static void lin_uart_process_fail(const struct device *dev) {
    struct lin_uart_data *data = dev->data;
    if (data->mode == LIN_MODE_COMMANDER) {
        if (data->state == LIN_UART_STATE_BREAK) {
            lin_uart_set_nominal_bitrate(dev);
        }
        if (data->sending && data->callbacks.tx) {
            data->callbacks.tx(dev, -EIO, data->callbacks.tx_data);
        } else if (!data->sending && data->callbacks.rx) {
            data->callbacks.rx(dev, -EIO, NULL, data->callbacks.rx_data);
        }
        data->state = LIN_UART_STATE_IDLE;
    } else { // LIN_MODE_RESPONDER
        if (data->callbacks.tx && (data->state == LIN_UART_STATE_SEND_RESPONSE ||
            data->state == LIN_UART_STATE_SEND_CHECKSUM)) {
            data->callbacks.tx(dev, -EIO, data->callbacks.tx_data);
        } else if (data->callbacks.rx && data->state == LIN_UART_STATE_RECV_RESPONSE) {
            data->callbacks.rx(dev, -EIO, NULL, data->callbacks.rx_data);
        }
        data->state = LIN_UART_STATE_BREAK;
    }
}

static void lin_uart_irq_handler(const struct device *uart_dev, void *lin_dev) {
    const struct device *dev = lin_dev;
    struct lin_uart_data *data = dev->data;
    if (k_sem_take(&data->irq_lock, K_NO_WAIT)) {
        lin_uart_clear_isr(dev);
        return; // timeout isr is handling already
    }
    if (!data->irq_has_lock && k_sem_take(&data->lock, K_NO_WAIT)) {
        lin_uart_clear_isr(dev);
        k_sem_give(&data->irq_lock);
        return; // currently configuring
    }
    data->irq_has_lock = true;
    k_timer_stop(&data->timer);

    bool spurious = true;
    while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
        /** NOTE: Common driver issue is that it clears errors even if there is no error.
         *  This behavior must be patched if present.
         */
        int err = uart_err_check(uart_dev);
        if (err > 0) {
            spurious = false;
            if (data->mode == LIN_MODE_RESPONDER && (err & (UART_ERROR_FRAMING | UART_BREAK))) {
                lin_uart_process_fail(dev); // call callbacks if packet was in progress
                k_timer_start(&data->timer, data->sync_timeout, K_FOREVER);
                data->state = LIN_UART_STATE_SYNC;
            } else {
                LOG_ERR("unexpected error byte %d", err);
                goto lin_uart_irq_fail;
            }
        }

        if (uart_irq_rx_ready(uart_dev)) {
            spurious = false;
            uint8_t bite = 0;
            if (uart_fifo_read(uart_dev, &bite, 1) <= 0) {
                LOG_ERR("fifo read not supported or no byte to read");
                goto lin_uart_irq_fail;
            }
            bool processed = true;
            if (data->mode == LIN_MODE_COMMANDER) {
                switch (data->state) {
                    case LIN_UART_STATE_BREAK:
                        if (bite != BREAK) {
                            LOG_ERR("break mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        lin_uart_set_nominal_bitrate(dev);
                        lin_uart_send_byte(dev, SYNC);
                        data->state = LIN_UART_STATE_SYNC;
                        break;

                    case LIN_UART_STATE_SYNC:
                        if (bite != SYNC) {
                            LOG_ERR("sync mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        lin_uart_send_byte(dev, PID[data->frame.id]);
                        data->state = LIN_UART_STATE_PID;
                        break;

                    case LIN_UART_STATE_PID:
                        uint8_t pid = PID[data->frame.id];
                        if (bite != pid) {
                            LOG_ERR("pid mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        data->frame_idx = 0;
                        if (data->sending) {
                            data->checksum = data->frame.type == LIN_CHECKSUM_ENHANCED ? pid : 0;
                            lin_uart_send_byte(dev, data->frame.data[0]);
                            lin_uart_update_checksum(data, data->frame.data[0]);
                            data->state = LIN_UART_STATE_SEND_RESPONSE;
                        } else {
                            data->checksum = 0;
                            data->state = LIN_UART_STATE_RECV_RESPONSE;
                            k_timer_start(&data->timer, data->timeout, K_FOREVER);
                        }
                        break;

                    case LIN_UART_STATE_SEND_RESPONSE:
                    case LIN_UART_STATE_SEND_CHECKSUM:
                    case LIN_UART_STATE_RECV_RESPONSE:
                        processed = false;
                        break;

                    default:
                        LOG_ERR("unexpected byte 0x%x for state %d", bite, data->state);
                        goto lin_uart_irq_fail;
                        break;
                }
            } else { // LIN_MODE_RESPONDER
                switch (data->state) {
                    case LIN_UART_STATE_SYNC:
                        if (bite == BREAK) { // break can arrive as a byte, try again
                            k_timer_start(&data->timer, data->sync_timeout, K_FOREVER);
                        } else {
                            if (bite != SYNC) {
                                LOG_ERR("invalid sync 0x%x", bite);
                                goto lin_uart_irq_fail;
                            }
                            k_timer_start(&data->timer, data->timeout, K_FOREVER);
                            data->state = LIN_UART_STATE_PID;
                        }
                        break;

                    case LIN_UART_STATE_PID:
                        data->frame.id = bite & LIN_ID_MASK;
                        if (bite != PID[data->frame.id]) {
                            LOG_ERR("invalid pid 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        enum lin_action action = LIN_ACTION_NONE;
                        if (data->callbacks.header) {
                            action = data->callbacks.header(dev, &data->frame, data->callbacks.header_data);
                        }
                        data->frame_idx = 0;
                        switch (action) {
                            case LIN_ACTION_SEND:
                                data->checksum = data->frame.type == LIN_CHECKSUM_ENHANCED ? bite : 0;
                                lin_uart_send_byte(dev, data->frame.data[0]);
                                lin_uart_update_checksum(data, data->frame.data[0]);
                                data->state = LIN_UART_STATE_SEND_RESPONSE;
                                break;

                            case LIN_ACTION_RECEIVE:
                                data->frame.id = bite & LIN_ID_MASK; // in case user sets it accidentally
                                data->checksum = 0;
                                k_timer_start(&data->timer, data->timeout, K_FOREVER);
                                data->state = LIN_UART_STATE_RECV_RESPONSE;
                                break;

                            default:
                                lin_uart_release_isr(data);
                                data->state = LIN_UART_STATE_BREAK; // spurious errors will be logged
                                break;
                        }
                        break;

                    case LIN_UART_STATE_SEND_RESPONSE:
                    case LIN_UART_STATE_SEND_CHECKSUM:
                    case LIN_UART_STATE_RECV_RESPONSE:
                        processed = false;
                        break;

                    default:
                        LOG_ERR("unexpected byte 0x%x for state %d", bite, data->state);
                        goto lin_uart_irq_fail;
                        break;
                }
            }

            // common logic
            if (!processed) {
                switch (data->state) {
                    case LIN_UART_STATE_SEND_RESPONSE:
                        if (bite != data->frame.data[data->frame_idx]) {
                            LOG_ERR("data %d mismatch 0x%x", data->frame_idx, bite);
                            goto lin_uart_irq_fail;
                        }
                        data->frame_idx++;
                        if (data->frame_idx < data->frame.len) {
                            lin_uart_send_byte(dev, data->frame.data[data->frame_idx]);
                            lin_uart_update_checksum(data, data->frame.data[data->frame_idx]);
                        } else {
                            lin_uart_send_byte(dev, ~data->checksum & 0xFF);
                            data->state = LIN_UART_STATE_SEND_CHECKSUM;
                        }
                        break;

                    case LIN_UART_STATE_SEND_CHECKSUM:
                        if (bite != (~data->checksum & 0xFF)) { // C type promotes!
                            LOG_ERR("checksum mismatch 0x%0x", bite);
                            goto lin_uart_irq_fail;
                        }
                        if (data->callbacks.tx) {
                            data->callbacks.tx(dev, 0, data->callbacks.tx_data);
                        }
                        lin_uart_release_isr(data);
                        data->state = LIN_UART_STATE_IDLE;
                        break;

                    case LIN_UART_STATE_RECV_RESPONSE:
                        if (data->frame_idx < LIN_MAX_DLEN) {
                            data->frame.data[data->frame_idx] = bite;
                        }
                        data->frame_idx++; // represents number of bytes received
                        lin_uart_update_checksum(data, bite);
                        bool must = (data->frame_idx >= 2 && data->frame_idx == (data->frame.len + 1)) ||
                                    (data->frame_idx > LIN_MAX_DLEN);
                        if (must || data->frame.len == 0) {
                            bool cc = data->checksum == 0xFF;
                            bool ec = (data->checksum + PID[data->frame.id]) == 0xFF; // optimization :)
                            bool match = false;
                            switch (data->frame.type) {
                                case LIN_CHECKSUM_AUTO:     match = cc || ec; break;
                                case LIN_CHECKSUM_CLASSIC:  match = cc;       break;
                                case LIN_CHECKSUM_ENHANCED: match = ec;       break;
                            }
                            if (match) {
                                if (data->frame.type == LIN_CHECKSUM_AUTO) {
                                    data->frame.type = cc ? LIN_CHECKSUM_CLASSIC : LIN_CHECKSUM_ENHANCED;
                                }
                                data->frame.len = data->frame_idx - 1;
                                if (data->callbacks.rx) {
                                    data->callbacks.rx(dev, 0, &data->frame, data->callbacks.rx_data);
                                }
                                lin_uart_release_isr(data);
                                data->state = LIN_UART_STATE_IDLE;
                            } else if (must) {
                                goto lin_uart_irq_fail;
                            } else {
                                k_timer_start(&data->timer, data->timeout, K_FOREVER);
                            }
                        } else {
                            k_timer_start(&data->timer, data->timeout, K_FOREVER);
                        }
                        break;

                    default:
                        break;
                }
            }
        }
    }
    if (spurious) {
        // STM32 and NXP drivers seem to cause lots of these...
        LOG_WRN("spurious interrupt, unknown cause");
        lin_uart_release_isr(data);
    }
    k_sem_give(&data->irq_lock);
    return;

lin_uart_irq_fail:
    lin_uart_clear_isr(dev);
    lin_uart_process_fail(dev);
    lin_uart_release_isr(data);
    k_sem_give(&data->irq_lock);
}

static void lin_uart_timeout_handler(struct k_timer *timer) {
    struct lin_uart_data *data = CONTAINER_OF(timer, struct lin_uart_data, timer);
    if (k_sem_take(&data->irq_lock, K_NO_WAIT)) {
        return; // uart isr is handling already
    }
    LOG_ERR("frame timeout in state %d", data->state);
    lin_uart_process_fail(data->lin_dev);
    lin_uart_release_isr(data);
    k_sem_give(&data->irq_lock);
}

/* API functions */
static int lin_uart_set_mode(const struct device *dev, enum lin_mode mode) {
    int ret = -EINVAL;
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    switch (mode) {
        case LIN_MODE_COMMANDER:
            ret = 0;
            data->mode = mode;
            data->state = LIN_UART_STATE_IDLE;
            break;

        case LIN_MODE_RESPONDER:
            ret = 0;
            data->mode = mode;
            data->state = LIN_UART_STATE_BREAK;
            break;
    }
    k_sem_give(&data->lock);
    return ret;
}

static int lin_uart_set_bitrate(const struct device *dev, uint32_t bitrate) {
    if (bitrate == 0 || bitrate > CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) {
        return -EINVAL; // NXP's driver has a divide by zero...
    }
    int ret = -EINVAL;
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    data->uart_cfg.baudrate = bitrate;
    if (uart_configure(cfg->uart_dev, &data->uart_cfg) == 0) {
        ret = 0;
        data->bitrate = bitrate;
        data->break_bitrate = bitrate * 9 / 13; // ~13 bit break
        if (cfg->max_wait_percent > 100) {
            // not LIN compliant, applies between individual bytes instead of header/response
            // setting timeout to double the desired percent because NXP driver needs it
            data->timeout = K_USEC(200000 * cfg->max_wait_percent / data->bitrate);
            data->break_timeout = K_USEC(200000 * cfg->max_wait_percent / data->break_bitrate);
            data->sync_timeout = K_USEC(200000 * cfg->max_wait_percent / data->bitrate);
        } else {
            data->timeout = K_FOREVER;
            data->break_timeout = K_FOREVER;
            data->sync_timeout = K_FOREVER;
        }
    }
    k_sem_give(&data->lock);
    return ret;
}

static int lin_uart_set_header_callback(const struct device *dev, lin_header_callback_t callback, void *user_data) {
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    data->callbacks.header = callback;
    data->callbacks.header_data = user_data;
    k_sem_give(&data->lock);
    return 0;
}

static int lin_uart_set_tx_callback(const struct device *dev, lin_tx_callback_t callback, void *user_data) {
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    data->callbacks.tx = callback;
    data->callbacks.tx_data = user_data;
    k_sem_give(&data->lock);
    return 0;
}

static int lin_uart_set_rx_callback(const struct device *dev, lin_rx_callback_t callback, void *user_data) {
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    data->callbacks.rx = callback;
    data->callbacks.rx_data = user_data;
    k_sem_give(&data->lock);
    return 0;
    
}

static int lin_uart_send(const struct device *dev, const struct lin_frame *frame) {
    if (frame->id >= LIN_NUM_ID || frame->len > LIN_MAX_DLEN || frame->len == 0) {
        return -EINVAL;
    }
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER); // importantly, can't be called in callbacks
    if (data->mode != LIN_MODE_COMMANDER) {
        k_sem_give(&data->lock);
        return -ENOTSUP;
    }
    data->irq_has_lock = true;
    data->sending = true;
    data->frame = *frame;
    lin_uart_send_break(dev);
    return 0;
}

static int lin_uart_receive(const struct device *dev, uint8_t id, enum lin_checksum type, uint8_t len) {
    if (id >= LIN_NUM_ID || len > LIN_MAX_DLEN) {
        return -EINVAL;
    }
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER); // importantly, can't be called in callbacks
    if (data->mode != LIN_MODE_COMMANDER) {
        k_sem_give(&data->lock);
        return -ENOTSUP;
    }
    data->irq_has_lock = true;
    data->sending = false;
    data->frame.id = id;
    data->frame.type = type;
    data->frame.len = len;
    lin_uart_send_break(dev);
    return 0;
}

static int lin_uart_init(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;

    if (!device_is_ready(cfg->uart_dev)) {
        return -ENODEV;
    }

    data->lin_dev = dev;

    k_sem_init(&data->lock, 1, 1);
    k_sem_init(&data->irq_lock, 1, 1);
    k_timer_init(&data->timer, lin_uart_timeout_handler, NULL);

    if (lin_uart_set_bitrate(dev, data->uart_cfg.baudrate)) {
        return -EINVAL;
    }

    uart_irq_callback_user_data_set(cfg->uart_dev, lin_uart_irq_handler, (void*) dev);
    uart_irq_rx_enable(cfg->uart_dev);
    uart_irq_err_enable(cfg->uart_dev);

    return 0;
}

static struct lin_driver_api lin_uart_api = {
    .set_mode            = lin_uart_set_mode,
    .set_bitrate         = lin_uart_set_bitrate,
    .set_header_callback = lin_uart_set_header_callback,
    .set_tx_callback     = lin_uart_set_tx_callback,
    .set_rx_callback     = lin_uart_set_rx_callback,
    .send                = lin_uart_send,
    .receive             = lin_uart_receive,
};

#define LIN_UART_INIT(n)                                        \
    static const struct lin_uart_config lin_uart_config_##n = { \
        .uart_dev         = DEVICE_DT_GET(DT_INST_PARENT(n)),   \
        .max_wait_percent = DT_INST_PROP(n, max_wait_percent),  \
    };                                                          \
                                                                \
    static struct lin_uart_data lin_uart_data_##n = {           \
        .mode  = LIN_MODE_COMMANDER,                            \
        .state = LIN_UART_STATE_IDLE,                           \
        .uart_cfg = {                                           \
            .baudrate  = DT_INST_PROP(n, bitrate),              \
            .parity    = UART_CFG_PARITY_NONE,                  \
            .stop_bits = UART_CFG_STOP_BITS_1,                  \
            .data_bits = UART_CFG_DATA_BITS_8,                  \
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,               \
        },                                                      \
    };                                                          \
                                                                \
    DEVICE_DT_INST_DEFINE(n,                                    \
                          &lin_uart_init,                       \
                          NULL,                                 \
                          &lin_uart_data_##n,                   \
                          &lin_uart_config_##n,                 \
                          POST_KERNEL,                          \
                          CONFIG_LIN_INIT_PRIORITY,             \
                          &lin_uart_api);

DT_INST_FOREACH_STATUS_OKAY(LIN_UART_INIT)
