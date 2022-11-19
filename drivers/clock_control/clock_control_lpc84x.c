#define DT_DRV_COMPAT nxp_lpc84x_syscon

#include <zephyr/devicetree.h>
#include <zephyr/device.h>

#include <zephyr/drivers/clock_control.h>
#include <fsl_clock.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control);

static int lpc84x_syscon_on(const struct device *dev,
                            clock_control_subsys_t sub_system) {
    clock_ip_name_t clock_ip_name = (clock_ip_name_t) sub_system;
    CLOCK_EnableClock(clock_ip_name);
    return 0;
}

static int lpc84x_syscon_off(const struct device *dev,
                             clock_control_subsys_t sub_system) {
    clock_ip_name_t clock_ip_name = (clock_ip_name_t) sub_system;
    CLOCK_DisableClock(clock_ip_name);
    return 0;
}

static int lpc84x_syscon_get_subsys_rate(const struct device *dev,
                                         clock_control_subsys_t sub_system,
                                         uint32_t *rate) {
    clock_name_t clock_name = (clock_name_t) sub_system;
    *rate = CLOCK_GetFreq(clock_name); // TODO not right API call
    return 0;
}

static int lpc84x_syscon_init(const struct device *dev) {
    return 0;
}

static const struct clock_control_driver_api lpc84x_syscon_driver_api = {
    .on = lpc84x_syscon_on,
    .off = lpc84x_syscon_off,
    .get_rate = lpc84x_syscon_get_subsys_rate,
};

DEVICE_DT_INST_DEFINE(0,
                      &lpc84x_syscon_init,
                      NULL,
                      NULL, NULL,
                      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
                      &lpc84x_syscon_driver_api);
