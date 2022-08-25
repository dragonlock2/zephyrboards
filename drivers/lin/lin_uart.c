#define DT_DRV_COMPAT zephyrboards_lin_uart

#include <zephyrboards/drivers/lin.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lin_uart, CONFIG_LIN_LOG_LEVEL);

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

/* private types */
#define LIN_UART_BREAK (0x00)
#define LIN_UART_SYNC  (0x55)

enum lin_uart_state {
    LIN_UART_STATE_IDLE,
    LIN_UART_STATE_BREAK,
    LIN_UART_STATE_SYNC,
    LIN_UART_STATE_PID,
    LIN_UART_STATE_SEND_RESPONSE,
    LIN_UART_STATE_RECV_RESPONSE,
};

struct lin_uart_txmsg {
    bool active;
    uint8_t pid;
    uint8_t data_len; // just data section, no checksum
    uint8_t res; // padding
    uint8_t data[LIN_MAX_DLEN + 1]; // one extra for checksum
    lin_tx_callback_t cb;
    void *cb_user_data;
    int retcode;
    struct k_sem lock;
};

struct lin_uart_rxfilt {
    bool active;
    uint8_t res[3]; // padding
    struct zlin_filter filter;
    lin_rx_callback_t cb;
    void *cb_user_data;
};

struct lin_uart_config {
    const struct device *uart_dev;
    const uint32_t max_wait_percent;
};

struct lin_uart_data {
    const struct device *lin_dev; // for timeout handler

    struct k_sem lock;
    bool irq_has_lock;

    uint32_t bitrate;
    struct uart_config uart_cfg;

    enum lin_mode mode;
    enum lin_uart_state state;

    struct lin_uart_txmsg tx_msgs[LIN_NUM_ID];
    struct lin_uart_rxfilt rx_filters[LIN_NUM_ID];
    struct lin_uart_txmsg *resp_msg; // responder mode, current message sent

    bool sending;
    struct lin_uart_txmsg msg; // also used for receiving
    uint8_t tx_idx;

    struct k_timer timer;
    k_timeout_t timeout;
    bool timing_out;
    bool timed_out;
};

/* private helpers */
static inline uint8_t lin_uart_checksum(struct lin_uart_txmsg *msg, enum lin_checksum_type type) {
    uint16_t sum = type == LIN_CHECKSUM_CLASSIC ? 0 : msg->pid; // default to enhanced
    for (int i = 0; i < msg->data_len; i++) {
        sum += msg->data[i];
        if (sum >= 256) {
            sum -= 255;
        }
    }
    return ~((uint8_t) (sum & 0xFF));
}

static inline bool lin_uart_checksum_match(struct lin_uart_txmsg *msg, enum lin_checksum_type type) {
    uint16_t sum = type == LIN_CHECKSUM_CLASSIC ? 0 : msg->pid; // default to enhanced
    for (int i = 0; i < msg->data_len; i++) {
        sum += msg->data[i];
        if (sum >= 256) {
            sum -= 255;
        }
    }
    return (sum + msg->data[msg->data_len]) == 0xFF;
}

static inline void lin_uart_send_break(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    data->uart_cfg.baudrate = data->bitrate * 9 / 13; // ~13 bit break
    if (uart_configure(cfg->uart_dev, &data->uart_cfg)) {
        LOG_ERR("unable to config for a break");
    }
    // preemptively grab lock to prevent race condition where send() and receive() called
    // consecutively before the break can be received
    if (k_sem_take(&data->lock, K_NO_WAIT) != 0) {
        LOG_ERR("unable to preemptively grab lock"); // should never happen
    }
    data->irq_has_lock = true;
    uart_poll_out(cfg->uart_dev, LIN_UART_BREAK);
    data->state = LIN_UART_STATE_BREAK;
    data->timed_out = false;
    k_timer_start(&data->timer, data->timeout, K_FOREVER);
}

/* NOTE: On some UART after a baudrate change, there's a latency. On STM32 and NXP,
 * it outputs one idle char at the new baudrate which puts us just within the 40%
 * margin LIN specifies.
 */
static inline void lin_uart_set_nominal_bitrate(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    data->uart_cfg.baudrate = data->bitrate;
    if (uart_configure(cfg->uart_dev, &data->uart_cfg)) {
        LOG_ERR("unable to config for nominal bitrate");
    }
}

static inline void lin_uart_clear_isr(const struct device *uart_dev) {
    while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
        if (uart_irq_rx_ready(uart_dev)) {
            uint8_t buffer[64];
            uart_err_check(uart_dev);
            uart_fifo_read(uart_dev, buffer, sizeof buffer);
        }
    }
}

static inline void lin_uart_release_isr(struct lin_uart_data *data) {
    data->irq_has_lock = false;
    k_sem_give(&data->lock);
}

static void lin_uart_irq_handler(const struct device *uart_dev, void *lin_dev) {
    const struct device *dev = lin_dev;
    struct lin_uart_data *data = dev->data;

    if (data->timing_out || (!data->irq_has_lock && k_sem_take(&data->lock, K_NO_WAIT) != 0)) {
        lin_uart_clear_isr(uart_dev);
        return;
    }

    bool spurious = true;
    data->irq_has_lock = true;
    while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
        if (uart_irq_rx_ready(uart_dev)) {
            spurious = false;
            // On MK22, doing the read first would actually clear the errors
            int err = uart_err_check(uart_dev);
            uint8_t bite = 0;
            if (uart_fifo_read(uart_dev, &bite, 1) < 0) {
                LOG_ERR("fifo read not supported");
                k_panic();
            }
            if (err > 0) {
                if (data->mode == LIN_MODE_RESPONDER &&
                    data->state == LIN_UART_STATE_BREAK &&
                    err & (UART_ERROR_FRAMING | UART_BREAK) &&
                    bite == LIN_UART_BREAK) { // does it always receive a 0 on break?
                    data->state = LIN_UART_STATE_SYNC;
                    data->timed_out = false;
                    k_timer_start(&data->timer, data->timeout, K_FOREVER);
                } else {
                    LOG_ERR("unexpected error byte");
                    goto lin_uart_irq_fail;
                }
            } else if (data->mode == LIN_MODE_COMMANDER) {
                switch (data->state) {
                    case LIN_UART_STATE_BREAK:
                        if (bite != LIN_UART_BREAK) {
                            LOG_ERR("break mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        lin_uart_set_nominal_bitrate(dev);
                        uart_poll_out(uart_dev, LIN_UART_SYNC);
                        data->state = LIN_UART_STATE_SYNC;
                        break;

                    case LIN_UART_STATE_SYNC:
                        if (bite != LIN_UART_SYNC) {
                            LOG_ERR("sync mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        uart_poll_out(uart_dev, data->msg.pid);
                        data->state = LIN_UART_STATE_PID;
                        break;

                    case LIN_UART_STATE_PID:
                        if (bite != data->msg.pid) {
                            LOG_ERR("pid mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        if (data->sending) {
                            data->tx_idx = 0;
                            uart_poll_out(uart_dev, data->msg.data[0]);
                            data->state = LIN_UART_STATE_SEND_RESPONSE;
                        } else {
                            data->msg.data_len = 0;
                            data->state = LIN_UART_STATE_RECV_RESPONSE;
                        }
                        break;

                    case LIN_UART_STATE_SEND_RESPONSE:
                        if (bite != data->msg.data[data->tx_idx]) {
                            LOG_ERR("data/checksum byte mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        data->tx_idx++;
                        if (data->tx_idx <= data->msg.data_len) { // <= bc of checksum
                            uart_poll_out(uart_dev, data->msg.data[data->tx_idx]);
                        } else {
                            k_timer_stop(&data->timer);
                            if (data->timed_out) {
                                goto lin_uart_irq_fail;
                            }
                            data->msg.retcode = 0;
                            if (data->msg.cb != NULL) {
                                data->msg.cb(dev, 0, data->msg.cb_user_data);
                            }
                            lin_uart_clear_isr(uart_dev);
                            lin_uart_release_isr(data);
                            data->state = LIN_UART_STATE_IDLE;
                        }
                        break;

                    case LIN_UART_STATE_RECV_RESPONSE:
                        data->msg.data[data->msg.data_len] = bite;
                        struct lin_uart_rxfilt *rxfilt = &data->rx_filters[data->msg.pid & LIN_ID_MASK];
                        struct zlin_filter *filter = &rxfilt->filter;

                        if (data->msg.data_len > 0 &&
                            (filter->data_len == 0 || data->msg.data_len == filter->data_len)) {
                            bool cc = false, ec = false;
                            switch (filter->checksum_type) {
                                case LIN_CHECKSUM_CLASSIC:
                                    cc = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_CLASSIC);
                                    break;

                                case LIN_CHECKSUM_ENHANCED:
                                    ec = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_ENHANCED);
                                    break;

                                case LIN_CHECKSUM_AUTO:
                                default:
                                    cc = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_CLASSIC);
                                    ec = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_ENHANCED);
                                    break;
                            }
                            if (cc || ec) {
                                k_timer_stop(&data->timer);
                                if (data->timed_out) {
                                    goto lin_uart_irq_fail;
                                }
                                struct zlin_frame msg = {
                                    .id = data->msg.pid & LIN_ID_MASK,
                                    .checksum_type = cc ? LIN_CHECKSUM_CLASSIC : LIN_CHECKSUM_ENHANCED,
                                    .data_len = data->msg.data_len,
                                };
                                memcpy(msg.data, data->msg.data, msg.data_len);
                                data->msg.retcode = 0;
                                if (data->msg.cb != NULL) {
                                    data->msg.cb(dev, 0, data->msg.cb_user_data);
                                }
                                if (rxfilt->cb != NULL) {
                                    rxfilt->cb(dev, &msg, rxfilt->cb_user_data);
                                }
                                lin_uart_clear_isr(uart_dev);
                                lin_uart_release_isr(data);
                                data->state = LIN_UART_STATE_IDLE;
                            } else if ((filter->data_len == 0 && data->msg.data_len >= LIN_MAX_DLEN) ||
                                       data->msg.data_len == filter->data_len) {
                                LOG_ERR("bad checksum");
                                goto lin_uart_irq_fail;
                            }
                        }
                        data->msg.data_len++;
                        break;

                    default:
                        LOG_ERR("unexpected byte for given state");
                        goto lin_uart_irq_fail;
                        break;
                }
            } else { // LIN_MODE_RESPONDER
                uint8_t id;
                switch (data->state) {
                    case LIN_UART_STATE_SYNC:
                        if (bite != LIN_UART_SYNC) {
                            LOG_ERR("invalid sync 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        data->state = LIN_UART_STATE_PID;
                        break;

                    case LIN_UART_STATE_PID:
                        id = bite & LIN_ID_MASK;
                        if (bite != PID[id]) {
                            LOG_ERR("invalid pid 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        if (data->tx_msgs[id].active) {
                            data->resp_msg = &data->tx_msgs[id];
                            data->resp_msg->active = false;
                            data->tx_idx = 0;
                            uart_poll_out(uart_dev, data->resp_msg->data[0]);
                            data->state = LIN_UART_STATE_SEND_RESPONSE;
                        } else if (data->rx_filters[id].active) {
                            data->msg.pid = bite;
                            data->msg.data_len = 0;
                            data->state = LIN_UART_STATE_RECV_RESPONSE;
                        } else {
                            k_timer_stop(&data->timer);
                            lin_uart_clear_isr(uart_dev);
                            lin_uart_release_isr(data);
                            data->state = LIN_UART_STATE_BREAK;
                        }
                        break;

                    case LIN_UART_STATE_SEND_RESPONSE:
                        if (bite != data->resp_msg->data[data->tx_idx]) {
                            LOG_ERR("data/checksum byte mismatch 0x%x", bite);
                            goto lin_uart_irq_fail;
                        }
                        data->tx_idx++;
                        if (data->tx_idx <= data->resp_msg->data_len) { // <= bc of checksum
                            uart_poll_out(uart_dev, data->resp_msg->data[data->tx_idx]);
                        } else {
                            k_timer_stop(&data->timer);
                            if (data->timed_out) {
                                goto lin_uart_irq_fail;
                            }
                            data->resp_msg->retcode = 0;
                            if (data->resp_msg->cb != NULL) {
                                data->resp_msg->cb(dev, 0, data->resp_msg->cb_user_data);
                            }
                            lin_uart_clear_isr(uart_dev);
                            lin_uart_release_isr(data);
                            k_sem_give(&data->resp_msg->lock);
                            data->state = LIN_UART_STATE_BREAK;
                        }
                        break;

                    case LIN_UART_STATE_RECV_RESPONSE:
                        data->msg.data[data->msg.data_len] = bite;
                        struct lin_uart_rxfilt *rxfilt = &data->rx_filters[data->msg.pid & LIN_ID_MASK];
                        struct zlin_filter *filter = &rxfilt->filter;

                        if (data->msg.data_len > 0 &&
                            (filter->data_len == 0 || data->msg.data_len == filter->data_len)) {
                            bool cc = false, ec = false;
                            switch (filter->checksum_type) {
                                case LIN_CHECKSUM_CLASSIC:
                                    cc = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_CLASSIC);
                                    break;

                                case LIN_CHECKSUM_ENHANCED:
                                    ec = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_ENHANCED);
                                    break;

                                case LIN_CHECKSUM_AUTO:
                                default:
                                    cc = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_CLASSIC);
                                    ec = lin_uart_checksum_match(&data->msg, LIN_CHECKSUM_ENHANCED);
                                    break;
                            }
                            if (cc || ec) {
                                k_timer_stop(&data->timer);
                                if (data->timed_out) {
                                    goto lin_uart_irq_fail;
                                }
                                struct zlin_frame msg = {
                                    .id = data->msg.pid & LIN_ID_MASK,
                                    .checksum_type = cc ? LIN_CHECKSUM_CLASSIC : LIN_CHECKSUM_ENHANCED,
                                    .data_len = data->msg.data_len,
                                };
                                memcpy(msg.data, data->msg.data, msg.data_len);
                                if (rxfilt->cb != NULL) {
                                    rxfilt->cb(dev, &msg, rxfilt->cb_user_data);
                                }
                                lin_uart_clear_isr(uart_dev);
                                lin_uart_release_isr(data);
                                k_sem_give(&data->msg.lock);
                                data->state = LIN_UART_STATE_BREAK;
                            } else if ((filter->data_len == 0 && data->msg.data_len >= LIN_MAX_DLEN) ||
                                       data->msg.data_len == filter->data_len) {
                                LOG_ERR("bad checksum");
                                goto lin_uart_irq_fail;
                            }
                        }
                        data->msg.data_len++;
                        break;

                    default:
                        LOG_ERR("unexpected byte for given state");
                        goto lin_uart_irq_fail;
                        break;
                }
            }
        }

        if (data->timed_out) {
            goto lin_uart_irq_fail;
        }
    }
    if (spurious) {
        if ((data->mode == LIN_MODE_COMMANDER && data->state == LIN_UART_STATE_IDLE) ||
            (data->mode == LIN_MODE_RESPONDER && data->state == LIN_UART_STATE_BREAK)) {
            // NXP driver seems to cause lots of these...
            LOG_ERR("spurious interrupt, unknown cause");
            lin_uart_release_isr(data);
        }
    }
    return;
lin_uart_irq_fail:
    k_timer_stop(&data->timer);
    if (data->timed_out) {
        if (data->mode == LIN_MODE_COMMANDER) {
            data->state = LIN_UART_STATE_IDLE;
        } else { // LIN_MODE_RESPONDER
            data->state = LIN_UART_STATE_BREAK;
        }
    } else {
        if (data->mode == LIN_MODE_COMMANDER) {
            if (data->state != LIN_UART_STATE_IDLE) {
                data->msg.retcode = -EIO;
                if (data->msg.cb != NULL) {
                    data->msg.cb(dev, -EIO, data->msg.cb_user_data);
                }
            }
            data->state = LIN_UART_STATE_IDLE;
        } else { // LIN_MODE_RESPONDER
            if (data->state == LIN_UART_STATE_SEND_RESPONSE) {
                data->resp_msg->active = false;
                data->resp_msg->retcode = -EIO;
                if (data->resp_msg->cb != NULL) {
                    data->resp_msg->cb(dev, -EIO, data->resp_msg->cb_user_data);
                }
                k_sem_give(&data->resp_msg->lock);
            }
            data->state = LIN_UART_STATE_BREAK;
        }
    }
    lin_uart_clear_isr(uart_dev);
    lin_uart_release_isr(data);
}

static inline void lin_uart_timeout_handler(struct k_timer *timer) {
    struct lin_uart_data *data = CONTAINER_OF(timer, struct lin_uart_data, timer);

    data->timing_out = true; // prevent UART IRQ from doing anything
    LOG_ERR("frame timeout, resetting to idle state");
    if (data->mode == LIN_MODE_COMMANDER) {
        if (data->state != LIN_UART_STATE_IDLE) {
            data->msg.retcode = -EIO;
            if (data->msg.cb != NULL) {
                data->msg.cb(data->lin_dev, -EIO, data->msg.cb_user_data);
            }
        }
        data->state = LIN_UART_STATE_IDLE;
    } else { // LIN_MODE_RESPONDER
        if (data->state == LIN_UART_STATE_SEND_RESPONSE) {
            data->resp_msg->active = false;
            data->resp_msg->retcode = -EIO;
            if (data->resp_msg->cb != NULL) {
                data->resp_msg->cb(data->lin_dev, -EIO, data->resp_msg->cb_user_data);
            }
            k_sem_give(&data->resp_msg->lock);
        }
        data->state = LIN_UART_STATE_BREAK;
    }
    lin_uart_release_isr(data);
    data->timing_out = false;
    data->timed_out = true; // signal back that UART IRQ was potentially interrupted
}

static inline void lin_uart_compute_timeout(const struct device *dev) {
    /* Technically non-conformant to LIN spec since it's just a worst case timeout that
     * measures header and response together.
     */
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;

    // 124 bits = 14+10+10+10*9 (break, sync, pid, data, checksum)
    data->timeout = K_USEC(10000 * 124 * cfg->max_wait_percent / data->bitrate);
}

static int lin_uart_init(const struct device *dev) {
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;

    if (!device_is_ready(cfg->uart_dev)) {
        return -ENODEV;
    }

    data->lin_dev = dev;

    k_sem_init(&data->lock, 1, 1);
    for (int i = 0; i < LIN_NUM_ID; i++) {
        k_sem_init(&data->tx_msgs[i].lock, 1, 1);
        data->tx_msgs[i].active = false;
        data->rx_filters[i].active = false;
    }

    data->timing_out = false;
    k_timer_init(&data->timer, lin_uart_timeout_handler, NULL);
    lin_uart_compute_timeout(dev);

    if (uart_configure(cfg->uart_dev, &data->uart_cfg)) {
        return -EIO;
    }
    uart_irq_callback_user_data_set(cfg->uart_dev, lin_uart_irq_handler, (void*) dev);
    uart_irq_rx_enable(cfg->uart_dev);
    uart_irq_err_enable(cfg->uart_dev);

    return 0;
}

/* API functions */
static int lin_uart_set_mode(const struct device *dev, enum lin_mode mode) {
    struct lin_uart_data *data = dev->data;
    int ret = 0;
    k_sem_take(&data->lock, K_FOREVER);
    switch (mode) {
        case LIN_MODE_COMMANDER:
            data->mode = LIN_MODE_COMMANDER;
            data->state = LIN_UART_STATE_IDLE;
            for (int i = 0; i < LIN_NUM_ID; i++) {
                if (data->tx_msgs[i].active) {
                    data->tx_msgs[i].active = false;
                    if (data->tx_msgs[i].cb != NULL) {
                        data->tx_msgs[i].cb(dev, -ECANCELED, data->tx_msgs[i].cb_user_data);
                    }
                    k_sem_give(&data->tx_msgs[i].lock);
                }
            }
            break;

        case LIN_MODE_RESPONDER:
            lin_uart_set_nominal_bitrate(dev);
            data->mode = LIN_MODE_RESPONDER;
            data->state = LIN_UART_STATE_BREAK;
            break;

        default:
            ret = -EINVAL;
            break;
    }
    k_sem_give(&data->lock);
    return ret;
}

static int lin_uart_set_bitrate(const struct device *dev, uint32_t bitrate) {
    if (bitrate == 0 || bitrate > CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) {
        return -EINVAL; // NXP's driver has a divide by zero...
    }
    const struct lin_uart_config *cfg = dev->config;
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    data->uart_cfg.baudrate = bitrate;
    if (uart_configure(cfg->uart_dev, &data->uart_cfg)) {
        k_sem_give(&data->lock);
        return -EIO;
    }
    data->bitrate = bitrate;
    lin_uart_compute_timeout(dev);
    k_sem_give(&data->lock);
    return 0;
}

static int lin_uart_send(const struct device *dev, const struct zlin_frame *frame,
                         k_timeout_t timeout, lin_tx_callback_t callback, void *user_data) {
    if (frame->id >= LIN_NUM_ID ||
        frame->data_len > LIN_MAX_DLEN ||
        frame->data_len == 0 ||
        (timeout.ticks != K_FOREVER.ticks && callback == NULL)) {
        return -EINVAL;
    }
    struct lin_uart_data *data = dev->data;
    if (data->mode == LIN_MODE_COMMANDER) {
        if (k_sem_take(&data->lock, timeout)) {
            return -EAGAIN;
        }
        data->sending          = true;
        data->msg.pid          = PID[frame->id];
        data->msg.data_len     = frame->data_len;
        data->msg.cb           = callback;
        data->msg.cb_user_data = user_data;
        memcpy(data->msg.data, frame->data, frame->data_len);
        data->msg.data[frame->data_len] = lin_uart_checksum(&data->msg, frame->checksum_type);
        k_sem_give(&data->lock);
        lin_uart_send_break(dev);
        return 0;
    } else { // LIN_MODE_RESPONDER
        struct lin_uart_txmsg *msg = &data->tx_msgs[frame->id];
        if (k_sem_take(&msg->lock, timeout)) {
            return -EAGAIN;
        }
        msg->pid          = PID[frame->id];
        msg->data_len     = frame->data_len;
        msg->cb           = callback;
        msg->cb_user_data = user_data;
        memcpy(msg->data, frame->data, frame->data_len);
        msg->data[frame->data_len] = lin_uart_checksum(msg, frame->checksum_type);
        msg->active = true;
        return 0;
    }
}

static int lin_uart_receive(const struct device *dev, uint8_t id, k_timeout_t timeout,
                            lin_tx_callback_t callback, void *user_data) {
    struct lin_uart_data *data = dev->data;
    if (id >= LIN_NUM_ID ||
        !data->rx_filters[id].active ||
        (timeout.ticks != K_FOREVER.ticks && callback == NULL)) {
        return -EINVAL;
    }
    if (data->mode != LIN_MODE_COMMANDER) {
        return -ENOTSUP;
    }
    if (k_sem_take(&data->lock, timeout)) {
        return -EAGAIN;
    }
    data->sending          = false;
    data->msg.pid          = PID[id];
    data->msg.cb           = callback;
    data->msg.cb_user_data = user_data;
    k_sem_give(&data->lock);
    lin_uart_send_break(dev);
    return 0;
}

static int lin_uart_add_rx_filter(const struct device *dev, lin_rx_callback_t callback,
                                  void *user_data, const struct zlin_filter *filter) {
    struct lin_uart_data *data = dev->data;
    if (filter->id >= LIN_NUM_ID || data->rx_filters[filter->id].active) {
        return -ENOSPC;
    }
    k_sem_take(&data->lock, K_FOREVER);
    data->rx_filters[filter->id].active = true;
    data->rx_filters[filter->id].filter = *filter;
    data->rx_filters[filter->id].cb = callback;
    data->rx_filters[filter->id].cb_user_data = user_data;
    k_sem_give(&data->lock);
    return filter->id;
}

static void lin_uart_remove_rx_filter(const struct device *dev, int filter_id) {
    if (filter_id >= LIN_NUM_ID) {
        return;
    }
    struct lin_uart_data *data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);
    data->rx_filters[filter_id].active = false;
    k_sem_give(&data->lock);
}

static struct lin_driver_api lin_uart_api = {
    .set_mode = lin_uart_set_mode,
    .set_bitrate = lin_uart_set_bitrate,
    .send = lin_uart_send,
    .receive = lin_uart_receive,
    .add_rx_filter = lin_uart_add_rx_filter,
    .remove_rx_filter = lin_uart_remove_rx_filter,
};

#define LIN_UART_INIT(n)                                        \
    static const struct lin_uart_config lin_uart_config_##n = { \
        .uart_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),           \
        .max_wait_percent = DT_INST_PROP(n, max_wait_percent),  \
    };                                                          \
                                                                \
    static struct lin_uart_data lin_uart_data_##n = {           \
        .irq_has_lock = false,                                  \
        .bitrate = DT_INST_PROP(n, bitrate),                    \
        .uart_cfg = {                                           \
            .baudrate  = DT_INST_PROP(n, bitrate),              \
            .parity    = UART_CFG_PARITY_NONE,                  \
            .stop_bits = UART_CFG_STOP_BITS_1,                  \
            .data_bits = UART_CFG_DATA_BITS_8,                  \
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,               \
        },                                                      \
        .mode = LIN_MODE_COMMANDER,                             \
        .state = LIN_UART_STATE_IDLE,                           \
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
