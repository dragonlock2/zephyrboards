#include "common.h"
#include <drivers/adc.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(test_adc)

static void adc_set_test(const struct device *adc, uint8_t *channels,
        size_t num_channels, uint32_t res, uint32_t ref, char *name) {
    LOG_INF("running test on %s", name);

    uint16_t samples[1];
    struct adc_channel_cfg cfg = {
        .gain             = ADC_GAIN_1,
        .reference        = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_MAX,
        .differential     = 0
    };
    struct adc_sequence seq = {
        .options      = NULL,
        .buffer       = samples,
        .buffer_size  = sizeof(samples),
        .resolution   = res,
        .oversampling = 0,
        .calibrate    = false
    };

    for (int i = 0; i < num_channels; i++) {
        cfg.channel_id = channels[i];
        adc_channel_setup(adc, &cfg);
        seq.channels = BIT(channels[i]);
        adc_read(adc, &seq);
        int32_t voltage = samples[0];
        adc_raw_to_millivolts(ref, ADC_GAIN_1, res, &voltage);
        LOG_INF("read %dmV on channel %d", voltage, channels[i]);
    }
}

#define RUN_ADC_SET(id)                                                          \
    const struct device *adc##id = DEVICE_DT_GET(DT_PROP(id, device));           \
    uint8_t chans##id[] = DT_PROP(id, channels);                                 \
    size_t num_chans##id = DT_PROP_LEN(id, channels);                            \
    uint32_t res##id = DT_PROP(id, resolution);                                  \
    uint32_t ref##id = DT_PROP(id, reference_voltage_mv);                        \
    char name##id[] = DT_PROP(id, label);                                        \
    adc_set_test(adc##id, chans##id, num_chans##id, res##id, ref##id, name##id); \

static int adc_test(const struct device *arg) {
    LOG_INF("starting ADC test");
    DT_FOREACH_STATUS_OKAY(test_adc, RUN_ADC_SET)
    return 0;
}

SYS_INIT(adc_test, APPLICATION, 6);

#endif // DT_HAS_COMPAT_STATUS_OKAY(test_adc)
