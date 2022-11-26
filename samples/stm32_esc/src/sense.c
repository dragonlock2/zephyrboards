#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include "sense.h"

/* private constants */
typedef struct {
    struct adc_dt_spec adc;
    uint32_t vref;
} sense_config_S;

static const sense_config_S SENSE_CONFIG[] = {
    [SENSE_CHANNEL_BEMF_A]   = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0), .vref = 36300},
    [SENSE_CHANNEL_BEMF_B]   = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1), .vref = 36300},
    [SENSE_CHANNEL_BEMF_C]   = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2), .vref = 36300},
    [SENSE_CHANNEL_ISENSE_A] = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3), .vref = 30000},
    [SENSE_CHANNEL_ISENSE_B] = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4), .vref = 30000},
    [SENSE_CHANNEL_ISENSE_C] = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 5), .vref = 30000},
    [SENSE_CHANNEL_VBUS]     = {.adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6), .vref = 36300},
};

/* public functions */
void sense_init() {
    for (sense_channel_E i = 0; i < SENSE_CHANNEL_COUNT; i++) {
        adc_channel_setup_dt(&SENSE_CONFIG[i].adc);
    }
}

uint32_t sense_read(sense_channel_E chan) {
    if (chan >= SENSE_CHANNEL_COUNT) { return 0; }
    int16_t val;
    struct adc_sequence seq = {
        .buffer = &val,
        .buffer_size = sizeof val,
    };
    adc_sequence_init_dt(&SENSE_CONFIG[chan].adc, &seq);
    adc_read(SENSE_CONFIG[chan].adc.dev, &seq);
    return (val * SENSE_CONFIG[chan].adc.vref_mv) >> SENSE_CONFIG[chan].adc.resolution;
}
