#define DT_DRV_COMPAT wch_ch32_ucpd

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ucpd_ch32, CONFIG_USBC_LOG_LEVEL);

// register fields copied from <ch32x035_usbpd.h>
#define USBPD_IN_HVT  (1 << 9)
#define USBPD_PHY_V33 (1 << 8)

#define CC_SEL        (1 << 2)

#define CC_PU_Mask    (3 << 2)
#define CC_NO_PU      (0 << 2)
#define CC_PU_330     (1 << 2)
#define CC_PU_180     (2 << 2)
#define CC_PU_80      (3 << 2)
#define CC_CMP_Mask   (7 << 5)
#define CC_NO_CMP     (0 << 5)
#define CC_CMP_22     (2 << 5)
#define CC_CMP_45     (3 << 5)
#define CC_CMP_55     (4 << 5)
#define CC_CMP_66     (5 << 5)
#define CC_CMP_95     (6 << 5)
#define CC_CMP_123    (7 << 5)
#define PA_CC_AI      (1 << 0)

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
    }
    return 0;
}

static int ucpd_ch32_receive_data(const struct device *dev, struct pd_msg *msg) {
    LOG_ERR("ucpd_ch32_receive_data"); // TODO filter SOP packets only?
    return -ENOSYS;
}

static int ucpd_ch32_transmit_data(const struct device *dev, struct pd_msg *msg) {
    LOG_ERR("ucpd_ch32_transmit_data"); // TODO return to rx right after
    return -ENOSYS;
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
    LOG_WRN("ucpd_ch32_set_roles not implemented");
    return -ENOSYS;
}

static void ucpd_ch32_isr(const struct device *dev) {
    LOG_ERR("isr %p", dev); // TODO impl, call alert cb
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
    return 0;
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
    struct ucpd_ch32_data *data = dev->data;

    config->irq_config_func();
    clock_control_on(config->clock_type, config->clock_mask);

    data->rp = TC_RP_USB;
    data->rp_enable = false;

    // TODO init into receiving state

    return 0;
}

static const struct tcpc_driver_api ucpd_ch32_driver_api = {
    .init                   = ucpd_ch32_init, // called twice
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
    .sop_prime_enable       = NULL,
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
