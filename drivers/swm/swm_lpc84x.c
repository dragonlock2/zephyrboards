#define DT_DRV_COMPAT nxp_lpc84x_swm

#include <devicetree.h>
#include <device.h>

#include <drivers/swm.h>
#include <drivers/clock_control.h>

#include <fsl_swm_connections.h>

#define LOG_LEVEL CONFIG_SWM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(swm);

struct lpc84x_swm_config {
    const char* clk_dev_name;
    uint32_t clk_name;
};

static int lpc84x_swm_assign(const struct device *dev,
                      swm_pin_t pin,
                      swm_function_t func) {
    // TODO
    return 0;
}

static int lpc84x_swm_init(const struct device *dev) {
    const struct lpc84x_swm_config *cfg = dev->config;

    clock_control_on(device_get_binding(cfg->clk_dev_name),
        (clock_control_subsys_t) cfg->clk_name);

    return 0;
}

static const struct swm_driver_api lpc84x_swm_driver_api = {
    .assign = lpc84x_swm_assign,
};

#define LPC84X_SWM_INIT(idx)                                              \
static const struct lpc84x_swm_config swm_cfg_##idx = {                   \
    .clk_dev_name = DT_LABEL(DT_PHANDLE(DT_NODELABEL(swm##idx), clocks)), \
    .clk_name = DT_PHA_BY_IDX(DT_NODELABEL(swm##idx), clocks, 0, name),   \
};                                                                        \
DEVICE_DT_DEFINE(DT_NODELABEL(swm##idx),                                  \
            &lpc84x_swm_init,                                             \
            NULL,                                                         \
            NULL, &swm_cfg_##idx,                                         \
            PRE_KERNEL_1, CONFIG_SWM_INIT_PRIORITY,                       \
            &lpc84x_swm_driver_api)

LPC84X_SWM_INIT(0);
