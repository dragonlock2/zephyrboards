#include "common.h"
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dac_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(test_dac)

static void dac_set_test(const struct device *dac, uint8_t *channels,
        size_t num_channels, uint32_t res, uint32_t ref, char *name) {
    LOG_INF("running test on %s", name);

    struct dac_channel_cfg cfg = {
        .resolution = res
    };

    for (int i = 0; i < num_channels; i++) {
        cfg.channel_id = channels[i];
        dac_channel_setup(dac, &cfg);
        dac_write_value(dac, channels[i], (1 << res) * 1000 / ref);
        LOG_INF("set channel %d to 1V", channels[i]);
    }
}

#define RUN_DAC_SET(id)                                                     \
    const struct device *dac##id = DEVICE_DT_GET(DT_PROP(id, device));      \
    uint8_t chans##id[] = DT_PROP(id, channels);                            \
    size_t num_chans##id = DT_PROP_LEN(id, channels);                       \
    uint32_t res##id = DT_PROP(id, resolution);                             \
    uint32_t ref##id = DT_PROP(id, reference_voltage_mv);                   \
    dac_set_test(dac##id, chans##id, num_chans##id, res##id, ref##id, #id); \

static int dac_test(const struct device *arg) {
    LOG_INF("starting DAC test");
    DT_FOREACH_STATUS_OKAY(test_dac, RUN_DAC_SET)
    return 0;
}

SYS_INIT(dac_test, APPLICATION, 7);

#endif // DT_HAS_COMPAT_STATUS_OKAY(test_dac)
