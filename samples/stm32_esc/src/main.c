#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyrboards/drivers/lin.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *lin = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_red),   gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue),  gpios),
};

const struct gpio_dt_spec dgs[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(dg1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(dg2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(dg3), gpios),
};

const struct gpio_dt_spec coast = GPIO_DT_SPEC_GET(DT_NODELABEL(coast), gpios);
const struct gpio_dt_spec sleep = GPIO_DT_SPEC_GET(DT_NODELABEL(sleep), gpios);
const struct gpio_dt_spec wdog  = GPIO_DT_SPEC_GET(DT_NODELABEL(wdog),  gpios);

// TODO replace w/ PWM
const struct gpio_dt_spec ih1 = GPIO_DT_SPEC_GET(DT_NODELABEL(ih1), gpios);
const struct gpio_dt_spec ih2 = GPIO_DT_SPEC_GET(DT_NODELABEL(ih2), gpios);
const struct gpio_dt_spec ih3 = GPIO_DT_SPEC_GET(DT_NODELABEL(ih3), gpios);
const struct gpio_dt_spec il1 = GPIO_DT_SPEC_GET(DT_NODELABEL(il1), gpios);
const struct gpio_dt_spec il2 = GPIO_DT_SPEC_GET(DT_NODELABEL(il2), gpios);
const struct gpio_dt_spec il3 = GPIO_DT_SPEC_GET(DT_NODELABEL(il3), gpios);

// BEMF comparator
const struct gpio_dt_spec bemf_trig[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(bemf_trig_a), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(bemf_trig_b), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(bemf_trig_c), gpios),
};

// BEMF direct measurement
const struct adc_dt_spec bemf_adc[] = {
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
};

// FOC current sense
const struct adc_dt_spec foc_adc[] = {
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 5),
};

int main() {
    lin_set_mode(lin, LIN_MODE_RESPONDER);
    lin_set_bitrate(lin, 19200);

    gpio_pin_configure_dt(&leds[0], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&leds[1], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&leds[2], GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&dgs[0], GPIO_INPUT);
    gpio_pin_configure_dt(&dgs[1], GPIO_INPUT);
    gpio_pin_configure_dt(&dgs[2], GPIO_INPUT);
    gpio_pin_configure_dt(&coast, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&sleep, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&wdog,  GPIO_OUTPUT_INACTIVE);

    // TODO replace w/ PWM
    gpio_pin_configure_dt(&ih1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&il1, GPIO_OUTPUT_ACTIVE);

    gpio_pin_configure_dt(&ih2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&il2, GPIO_OUTPUT_ACTIVE);

    gpio_pin_configure_dt(&ih3, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&il3, GPIO_OUTPUT_ACTIVE);

    // BEMF comparator
    gpio_pin_configure_dt(&bemf_trig[0], GPIO_INPUT);
    gpio_pin_configure_dt(&bemf_trig[1], GPIO_INPUT);
    gpio_pin_configure_dt(&bemf_trig[2], GPIO_INPUT);

    // BEMF direct measurement
    for (int i = 0; i < 3; i++) {
        adc_channel_setup_dt(&bemf_adc[i]);
    }

    // FOC current sense
    for (int i = 0; i < 3; i++) {
        adc_channel_setup_dt(&foc_adc[i]);
    }

    LOG_DBG("booted!");
    while (1) {
        LOG_DBG("bemf trig %d %d %d",
            gpio_pin_get_dt(&bemf_trig[0]),
            gpio_pin_get_dt(&bemf_trig[1]),
            gpio_pin_get_dt(&bemf_trig[2])
        );

        int16_t val;
        struct adc_sequence adc_seq = {
            .buffer = &val,
            .buffer_size = sizeof val,
        };

        int32_t bemf_mv[3];
        for (int i = 0; i < 3; i++) {
            adc_sequence_init_dt(&bemf_adc[i], &adc_seq);
            adc_read(bemf_adc[i].dev, &adc_seq);
            bemf_mv[i] = val;
            adc_raw_to_millivolts_dt(&bemf_adc[i], &bemf_mv[i]);
        }
        LOG_DBG("bemf mV %d %d %d", bemf_mv[0], bemf_mv[1], bemf_mv[2]);

        int32_t foc_ma[3];
        for (int i = 0; i < 3; i++) {
            adc_sequence_init_dt(&foc_adc[i], &adc_seq);
            adc_read(foc_adc[i].dev, &adc_seq);
            foc_ma[i] = (val * 30000) >> 12; // can't set separate reference
        }
        LOG_DBG("foc mA %d %d %d", foc_ma[0], foc_ma[1], foc_ma[2]);

        gpio_pin_toggle_dt(&leds[2]);
        k_msleep(500);
    }
    return 0;
}
