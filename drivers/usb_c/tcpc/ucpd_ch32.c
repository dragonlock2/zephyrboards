#define DT_DRV_COMPAT wch_ch32_ucpd

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ucpd_ch32, CONFIG_USBC_LOG_LEVEL);

struct ucpd_ch32_config {
    USBPD_TypeDef *base;
    uint32_t clock_type, clock_mask;
    void (*irq_config_func)(void);
};

struct ucpd_ch32_data {
    tcpc_alert_handler_cb_t alert_cb;
    void *alert_data;
    tcpc_vconn_control_cb_t vconn_cb;
    tcpc_vconn_discharge_cb_t vconn_discharge_cb;
    enum tc_cc_polarity polarity;
    enum tc_rp_value rp;
    bool rp_enable;
    bool rx_sop_prime;
    enum tc_power_role power_role;
    enum tc_data_role data_role;
    __attribute__((aligned(4))) uint8_t buffer[512];
    struct k_sem tx_lock;
    bool tx_good_crc;
    bool tx_need_crc;
    bool tx_valid;
    struct k_sem rx_msg_lock;
    struct pd_msg rx_msg;
    bool rx_msg_valid;
};

static void ucpd_ch32_update_rp(const struct device *dev) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;
    uint16_t source = CC_NO_PU;
    if (data->rp_enable) {
        switch (data->rp) {
            case TC_RP_1A5:
                source = CC_PU_180;
                break;
            case TC_RP_3A0:
                source = CC_PU_330;
                break;
            default:
                source = CC_PU_80;
                break;
        }
    }
    config->base->PORT_CC1 = (config->base->PORT_CC1 & ~CC_PU_Mask) | source;
    config->base->PORT_CC2 = (config->base->PORT_CC2 & ~CC_PU_Mask) | source;
}

static void ucpd_ch32_notify_alert(const struct device *dev, enum tcpc_alert alert) {
    struct ucpd_ch32_data *data = dev->data;
    if (data->alert_cb) {
        data->alert_cb(dev, data->alert_data, alert);
    }
}

static void ucpd_ch32_start_rx(const struct device *dev) {
    const struct ucpd_ch32_config *config = dev->config;
    config->base->CONFIG      |= PD_ALL_CLR;
    config->base->CONFIG      &= ~PD_ALL_CLR;
    config->base->CONTROL     &= ~PD_TX_EN;
    config->base->BMC_CLK_CNT  = UPD_TMR_RX_48M;
    config->base->CONTROL     |= BMC_START;
}

static void ucpd_ch32_start_tx(const struct device *dev, enum pd_packet_type type,
        union pd_header header, uint8_t *buffer, uint32_t len) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;
    
    // setup packet
    len = MIN(len, sizeof(data->buffer) - 2);
    switch (type) {
        case PD_PACKET_SOP:           config->base->TX_SEL = UPD_SOP0;        data->tx_need_crc = true;  break;
        case PD_PACKET_SOP_PRIME:     config->base->TX_SEL = UPD_SOP1;        data->tx_need_crc = true;  break;
        case PD_PACKET_PRIME_PRIME:   config->base->TX_SEL = UPD_SOP2;        data->tx_need_crc = true;  break;
        case PD_PACKET_TX_HARD_RESET: config->base->TX_SEL = UPD_HARD_RESET;  data->tx_need_crc = false; break;
        case PD_PACKET_CABLE_RESET:   config->base->TX_SEL = UPD_CABLE_RESET; data->tx_need_crc = false; break;
        default:
            LOG_WRN("unsupported packet type %d, default to SOP", type);
            data->tx_need_crc = true;
            config->base->TX_SEL = UPD_SOP0;
            break;
    }
    config->base->BMC_TX_SZ = 2 + len;
    data->buffer[0] = (header.raw_value >> 0) & 0xFF;
    data->buffer[1] = (header.raw_value >> 8) & 0xFF;
    memcpy(&data->buffer[2], buffer, len);
    data->tx_valid = true;

    // send packet
    if (data->polarity == TC_POLARITY_CC1) {
        config->base->PORT_CC1 |= CC_LVE;
    } else { // TC_POLARITY_CC2
        config->base->PORT_CC2 |= CC_LVE;
    }
    config->base->BMC_CLK_CNT  = UPD_TMR_TX_48M;
    config->base->CONTROL     |= PD_TX_EN;
    config->base->STATUS      &= BMC_AUX_INVALID;
    config->base->CONTROL     |= BMC_START;
}

static void ucpd_ch32_end_tx(const struct device *dev) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;
    if (data->polarity == TC_POLARITY_CC1) {
        config->base->PORT_CC1 &= ~CC_LVE;
    } else { // TC_POLARITY_CC2
        config->base->PORT_CC2 &= ~CC_LVE;
    }
}

static bool ucpd_ch32_is_good_crc(union pd_header header) {
    return header.number_of_data_objects == 0 &&
           header.extended == 0 &&
           header.message_type == PD_CTRL_GOOD_CRC;
}

static int ucpd_ch32_set_alert_handler_cb(const struct device *dev,
        tcpc_alert_handler_cb_t handler, void *alert_data) {
    struct ucpd_ch32_data *data = dev->data;
    data->alert_cb = handler;
    data->alert_data = alert_data;
    return 0;
}

static int ucpd_ch32_get_cc(const struct device *dev,
        enum tc_cc_voltage_state *cc1, enum tc_cc_voltage_state *cc2) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;

    const struct {
        volatile uint16_t * const reg;
        enum tc_cc_voltage_state * const state;
    } ccs[2] = {
        { &config->base->PORT_CC1, cc1},
        { &config->base->PORT_CC2, cc2},
    };

    struct ucpd_ch32_comp {
        uint16_t thresh;
        enum tc_cc_voltage_state state;
    };
    const struct ucpd_ch32_comp RP_USB_COMPS[1] = { 
        { CC_CMP_22, TC_CC_VOLT_RD }, // 80uA @ 5.1kΩ
        // 80uA @ 1kΩ is 0.08V, impossible to detect
    };
    const struct ucpd_ch32_comp RP_1A5_COMPS[1] = { 
        { CC_CMP_55, TC_CC_VOLT_RD }, // 180uA @ 5.1kΩ
        // 180uA @ 1kΩ is 0.18V, impossible to detect
    };
    const struct ucpd_ch32_comp RP_3A0_COMPS[2] = { 
        { CC_CMP_95, TC_CC_VOLT_RD }, // 330uA @ 5.1kΩ
        { CC_CMP_22, TC_CC_VOLT_RA }, // 330uA @ 1kΩ
    };
    const struct ucpd_ch32_comp RD_COMPS[3] = {
        { CC_CMP_123, TC_CC_VOLT_RP_3A0 }, // 330uA @ 5.1kΩ
        { CC_CMP_66,  TC_CC_VOLT_RP_1A5 }, // 180uA @ 5.1kΩ
        { CC_CMP_22,  TC_CC_VOLT_RP_DEF }, //  80uA @ 5.1kΩ
    };
    const struct ucpd_ch32_comp *comps;
    size_t num_comps;

    for (int i = 0; i < ARRAY_SIZE(ccs); i++) {
        if (data->rp_enable) {
            switch (data->rp) {
                case TC_RP_1A5:
                    comps = RP_1A5_COMPS;
                    num_comps = ARRAY_SIZE(RP_1A5_COMPS);
                    break;
                case TC_RP_3A0:
                    comps = RP_3A0_COMPS;
                    num_comps = ARRAY_SIZE(RP_3A0_COMPS);
                    LOG_WRN("can't detect ra with current rp");
                    break;
                default:
                    comps = RP_USB_COMPS;
                    num_comps = ARRAY_SIZE(RP_USB_COMPS);
                    LOG_WRN("can't detect ra with current rp");
                    break;
            }
        } else {
            comps = RD_COMPS;
            num_comps = ARRAY_SIZE(RD_COMPS);
        }
        *ccs[i].state = TC_CC_VOLT_OPEN;
        for (int j = 0; j < num_comps; j++) {
            *ccs[i].reg = (*ccs[i].reg & ~CC_CMP_Mask) | comps[j].thresh;
            k_busy_wait(2);
            if (*ccs[i].reg & PA_CC_AI) {
                *ccs[i].state = comps[j].state;
                break;
            }
        }

        // need to reset comparator for reliable receiving
        *ccs[i].reg = (*ccs[i].reg & ~CC_CMP_Mask) | CC_CMP_66;
    }
    return 0;
}

static int ucpd_ch32_receive_data(const struct device *dev, struct pd_msg *msg) {
    int ret = -EIO;
    struct ucpd_ch32_data *data = dev->data;
    if (k_sem_take(&data->rx_msg_lock, K_NO_WAIT) == 0) {
        if (data->rx_msg_valid) {
            *msg = data->rx_msg;
            ret = data->rx_msg.len + 2; // usbc_prl.c checks > 0
            data->rx_msg_valid = false;
        }
        k_sem_give(&data->rx_msg_lock);
    }
    return ret;
}

static int ucpd_ch32_transmit_data(const struct device *dev, struct pd_msg *msg) {
    struct ucpd_ch32_data *data = dev->data;
    if (k_sem_take(&data->tx_lock, K_MSEC(10))) {
        LOG_ERR("timeout waiting for transmit done");
        return -EIO;
    } else {
        data->tx_good_crc = false;
        ucpd_ch32_start_tx(dev, msg->type, msg->header, msg->data, msg->len);
        return 0;
    }
}

static int ucpd_ch32_select_rp_value(const struct device *dev, enum tc_rp_value rp) {
    struct ucpd_ch32_data *data = dev->data;
    data->rp = rp;
    ucpd_ch32_update_rp(dev);
    return 0;
}

static int ucpd_ch32_get_rp_value(const struct device *dev, enum tc_rp_value *rp) {
    struct ucpd_ch32_data *data = dev->data;
    *rp = data->rp;
    return 0;
}

static int ucpd_ch32_set_cc(const struct device *dev, enum tc_cc_pull pull) {
    struct ucpd_ch32_data *data = dev->data;
    switch (pull) {
        case TC_CC_RA:
        case TC_CC_RD:
        case TC_CC_OPEN:
        case TC_RA_RD:
            LOG_WRN("dynamically setting pull %d not supported", pull);
            data->rp_enable = false;
            break;

        case TC_CC_RP:
            data->rp_enable = true;
            break;
    }
    ucpd_ch32_update_rp(dev);
    return 0;
}

static int ucpd_ch32_set_roles(const struct device *dev,
        enum tc_power_role power_role, enum tc_data_role data_role) {
    struct ucpd_ch32_data *data = dev->data;
    data->power_role = power_role;
    data->data_role = data_role;
    if (data->data_role == TC_ROLE_DFP) {
        LOG_WRN("USB driver doesn't support host mode yet");
    }
    return 0;
}

static void ucpd_ch32_set_vconn_cb(const struct device *dev, tcpc_vconn_control_cb_t vconn_cb) {
    struct ucpd_ch32_data *data = dev->data;
    data->vconn_cb = vconn_cb;
}

static void ucpd_ch32_set_vconn_discharge_cb(const struct device *dev, tcpc_vconn_discharge_cb_t cb) {
    struct ucpd_ch32_data *data = dev->data;
    data->vconn_discharge_cb = cb;
}

static int ucpd_ch32_set_vconn(const struct device *dev, bool enable) {
    if (enable) {
        LOG_WRN("ucpd_ch32_set_vconn enable not implemented");
    }
    return -ENOSYS;
}

static int ucpd_ch32_vconn_discharge(const struct device *dev, bool enable) {
    if (enable) {
        LOG_WRN("ucpd_ch32_vconn_discharge enable not implemented");
    }
    return -ENOSYS;
}

static int ucpd_ch32_set_cc_polarity(const struct device *dev, enum tc_cc_polarity polarity) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;
    data->polarity = polarity;
    if (data->polarity == TC_POLARITY_CC1) {
        config->base->CONFIG = config->base->CONFIG & ~CC_SEL;
    } else { // TC_POLARITY_CC2
        config->base->CONFIG = config->base->CONFIG | CC_SEL;
    }
    ucpd_ch32_start_rx(dev); // don't start receive until after polarity set
    return 0;
}

static int ucpd_ch32_sop_prime_enable(const struct device *dev, bool enable) {
    struct ucpd_ch32_data *data = dev->data;
    data->rx_sop_prime = enable;
    return 0;
}

static void ucpd_ch32_isr(const struct device *dev) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;
    uint8_t status = config->base->STATUS;
    if (status & IF_RX_ACT) {
        uint16_t byte_count = config->base->BMC_BYTE_CNT;
        if ((byte_count < (2 + 4)) || (byte_count > (PD_MAX_EXTENDED_MSG_LEN + 2 + 4))) {
            LOG_ERR("invalid packet length");
        } else {
            if (k_sem_take(&data->rx_msg_lock, K_NO_WAIT)) {
                ucpd_ch32_notify_alert(dev, TCPC_ALERT_RX_BUFFER_OVERFLOW);
                LOG_ERR("message overrun, dropping new packet");
            } else {
                if (data->rx_msg_valid) {
                    ucpd_ch32_notify_alert(dev, TCPC_ALERT_RX_BUFFER_OVERFLOW);
                    LOG_ERR("message overrun, replacing old packet");
                }
                switch (status & MASK_PD_STAT) {
                    case PD_RX_SOP0:      data->rx_msg.type = PD_PACKET_SOP;         break;
                    case PD_RX_SOP1_HRST: data->rx_msg.type = PD_PACKET_SOP_PRIME;   break;
                    case PD_RX_SOP2_CRST: data->rx_msg.type = PD_PACKET_PRIME_PRIME; break;
                    default:              data->rx_msg.type = PD_PACKET_MSG_INVALID; break;
                }
                data->rx_msg.header.raw_value = (data->buffer[1] << 8) | data->buffer[0];
                data->rx_msg.len = byte_count - 2 - 4; // remove header and CRC
                memcpy(data->rx_msg.data, &data->buffer[2], data->rx_msg.len);
                if (!ucpd_ch32_is_good_crc(data->rx_msg.header) &&
                    (data->rx_msg.type == PD_PACKET_SOP || data->rx_sop_prime)) {
                    if (k_sem_take(&data->tx_lock, K_NO_WAIT)) {
                        LOG_ERR("transmit in progress, dropping GoodCRC"); // shouldn't happen
                    } else {
                        union pd_header header = {
                            .message_type           = PD_CTRL_GOOD_CRC,
                            .port_data_role         = 0,
                            .specification_revision = data->rx_msg.header.specification_revision,
                            .port_power_role        = 0,
                            .message_id             = data->rx_msg.header.message_id,
                            .number_of_data_objects = 0,
                            .extended               = 0,
                        };
                        if (data->rx_msg.type == PD_PACKET_SOP) {
                            header.port_power_role = data->power_role;
                            header.port_data_role  = data->data_role;
                        }
                        data->tx_good_crc = true;
                        ucpd_ch32_start_tx(dev, data->rx_msg.type, header, NULL, 0);
                    }
                    // hold lock until GoodCRC sent
                } else {
                    if (data->tx_valid && ucpd_ch32_is_good_crc(data->rx_msg.header)) {
                        data->tx_valid = false;
                        k_sem_give(&data->tx_lock);
                        ucpd_ch32_notify_alert(dev, TCPC_ALERT_TRANSMIT_MSG_SUCCESS); // TODO add timeouts and retries
                    }
                    data->rx_msg_valid = true;
                    k_sem_give(&data->rx_msg_lock);
                    ucpd_ch32_notify_alert(dev, TCPC_ALERT_MSG_STATUS);
                    // no GoodCRC needed, send to stack immediately
                }
            }
        }
        config->base->STATUS |= IF_RX_ACT;
    }
    if (status & IF_TX_END) {
        ucpd_ch32_end_tx(dev);
        ucpd_ch32_start_rx(dev);
        if (data->tx_good_crc) {
            data->tx_valid = false;
            data->rx_msg_valid = true;
            k_sem_give(&data->tx_lock);
            k_sem_give(&data->rx_msg_lock);
            ucpd_ch32_notify_alert(dev, TCPC_ALERT_MSG_STATUS);
        } else if (!data->tx_need_crc) {
            data->tx_valid = false;
            k_sem_give(&data->tx_lock);
            ucpd_ch32_notify_alert(dev, TCPC_ALERT_TRANSMIT_MSG_SUCCESS);
        }
        config->base->STATUS |= IF_TX_END;
    }
    if (status & IF_RX_RESET) {
        ucpd_ch32_notify_alert(dev, TCPC_ALERT_HARD_RESET_RECEIVED);
        config->base->STATUS |= IF_RX_RESET;
    }
}

static int ucpd_ch32_init(const struct device *dev) {
#if IS_ENABLED(CONFIG_SOC_CH32X035)
    if (PWR_VDD_SupplyVoltage() == PWR_VDD_5V) {
        AFIO->CTLR = (AFIO->CTLR & ~USBPD_PHY_V33) | USBPD_IN_HVT;
    } else {
        AFIO->CTLR = (AFIO->CTLR) | USBPD_IN_HVT | USBPD_PHY_V33;
    }

    GPIO_InitTypeDef cc_cfg = {
        .GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_15,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
    };
    GPIO_Init(GPIOC, &cc_cfg);
#else
    #error "update driver to init USB PD GPIO for this SoC"
#endif

    const struct ucpd_ch32_config *config = dev->config;
    config->irq_config_func();
    clock_control_on(config->clock_type, config->clock_mask);

    return 0;
}

static int ucpd_ch32_driver_init(const struct device *dev) {
    const struct ucpd_ch32_config *config = dev->config;
    struct ucpd_ch32_data *data = dev->data;

    k_sem_init(&data->tx_lock, 1, 1);
    k_sem_init(&data->rx_msg_lock, 1, 1);

    data->rp           = TC_RP_USB;
    data->rp_enable    = false;
    data->rx_sop_prime = false;
    data->power_role   = TC_ROLE_SINK;
    data->data_role    = TC_ROLE_UFP;
    data->tx_valid     = false;

    config->base->STATUS = 0xFC; // clear interrupts
    config->base->CONFIG = PD_ALL_CLR;
    config->base->CONFIG = 0;
    config->base->DMA    = (uint32_t) data->buffer;
    config->base->CONFIG = IE_TX_END | IE_RX_RESET | IE_RX_ACT | PD_DMA_EN;

    return 0;
}

static const struct tcpc_driver_api ucpd_ch32_driver_api = {
    .init                   = ucpd_ch32_driver_init,
    .set_alert_handler_cb   = ucpd_ch32_set_alert_handler_cb,
    .get_cc                 = ucpd_ch32_get_cc,
    .set_rx_enable          = NULL,
    .is_rx_pending_msg      = NULL,
    .receive_data           = ucpd_ch32_receive_data,
    .transmit_data          = ucpd_ch32_transmit_data,
    .select_rp_value        = ucpd_ch32_select_rp_value,
    .get_rp_value           = ucpd_ch32_get_rp_value,
    .set_cc                 = ucpd_ch32_set_cc,
    .set_roles              = ucpd_ch32_set_roles,
    .set_vconn_cb           = ucpd_ch32_set_vconn_cb,
    .set_vconn_discharge_cb = ucpd_ch32_set_vconn_discharge_cb,
    .set_vconn              = ucpd_ch32_set_vconn,
    .vconn_discharge        = ucpd_ch32_vconn_discharge,
    .set_cc_polarity        = ucpd_ch32_set_cc_polarity,
    .dump_std_reg           = NULL,
    .set_bist_test_mode     = NULL,
    .sop_prime_enable       = ucpd_ch32_sop_prime_enable,
};

#define UCPD_CH32_INIT(n)                                           \
    static void ucpd_ch32_##n##_irq_config(void) {                  \
        IRQ_CONNECT(DT_INST_PROP_BY_IDX(n, irq, 0), 0,              \
            ucpd_ch32_isr, DEVICE_DT_INST_GET(n), 0);               \
        irq_enable(DT_INST_PROP_BY_IDX(n, irq, 0));                 \
    }                                                               \
                                                                    \
    static const struct ucpd_ch32_config ucpd_ch32_##n##_config = { \
        .base            = (USBPD_TypeDef*) DT_INST_REG_ADDR(n),    \
        .clock_type      = DT_INST_PROP_BY_IDX(n, clk, 0),          \
        .clock_mask      = DT_INST_PROP_BY_IDX(n, clk, 1),          \
        .irq_config_func = ucpd_ch32_##n##_irq_config,              \
    };                                                              \
                                                                    \
    static struct ucpd_ch32_data ucpd_ch32_##n##_data;              \
                                                                    \
    DEVICE_DT_INST_DEFINE(n,                                        \
        &ucpd_ch32_init,                                            \
        NULL,                                                       \
        &ucpd_ch32_##n##_data,                                      \
        &ucpd_ch32_##n##_config,                                    \
        POST_KERNEL,                                                \
        CONFIG_USBC_TCPC_INIT_PRIORITY,                             \
        &ucpd_ch32_driver_api                                       \
    );

DT_INST_FOREACH_STATUS_OKAY(UCPD_CH32_INIT)
