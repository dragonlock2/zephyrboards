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

struct ucpd_ch32_config {
    USBPD_TypeDef *base;
    uint32_t clock_type, clock_mask;
    void (*irq_config_func)(void);
};

struct ucpd_ch32_data {
    // TODO add stuff, probs CC
};

static void ucpd_ch32_isr(const struct device *dev) {
    LOG_ERR("isr %p", dev); // TODO impl, call cb
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

    return 0;
}

static const struct tcpc_driver_api ucpd_ch32_driver_api = {
    .init                   = ucpd_ch32_init, // called twice
    .set_alert_handler_cb   = NULL, // TODO
    .get_cc                 = NULL, // TODO
    .set_rx_enable          = NULL,
    .is_rx_pending_msg      = NULL,
    .receive_data           = NULL, // TODO filter only SOP?
    .transmit_data          = NULL, // TODO return to rx right after
    .select_rp_value        = NULL, // TODO only apply if RP applied
    .get_rp_value           = NULL, // TODO
    .set_cc                 = NULL, // TODO
    .set_roles              = NULL, // TODO
    .set_vconn_cb           = NULL, // TODO set cb
    .set_vconn_discharge_cb = NULL, // TODO set cb
    .set_vconn              = NULL, // TODO call cb
    .vconn_discharge        = NULL, // TODO call cb
    .set_cc_polarity        = NULL, // TODO selects CC
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
