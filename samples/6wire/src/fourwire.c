#include "fourwire.h"

#include <drivers/i2c.h>
#include <math.h>
#include <stdlib.h>

K_THREAD_STACK_DEFINE(fourwire_stack_area, 1024);
static struct k_thread fourwire_thread_data;

K_MSGQ_DEFINE(fourwire_msgq, sizeof(fourwire_config_t*), 8, 4);

void fourwire_init(fourwire_config_t *cfg) {
    // mux setup
    for (int i = 0; i < cfg->num_mux_pins; i++) {
        gpio_pin_configure_dt(&cfg->mux_pins[i], GPIO_OUTPUT_INACTIVE);
    }

    cfg->best_idx = cfg->num_ref_vals - 1;
    fourwire_set_ref(cfg, cfg->best_idx);

    // ADC setup
    uint8_t cmd = 0x06; // RESET
    i2c_write(cfg->adc_i2c, &cmd, 1, cfg->adc_addr);
    // input to AIN0/AIN1, gain=1, PGA enabled
    i2c_reg_write_byte(cfg->adc_i2c, cfg->adc_addr, 0x40, 0x00);
    // 20SPS, normal mode, continuous, REFP/REFN ref, no temp
    i2c_reg_write_byte(cfg->adc_i2c, cfg->adc_addr, 0x44, 0x0a);
    // no data counter, no data check, burnt-out current source off, IDAC=0A
    i2c_reg_write_byte(cfg->adc_i2c, cfg->adc_addr, 0x48, 0x00);
    // IDACs disabled
    i2c_reg_write_byte(cfg->adc_i2c, cfg->adc_addr, 0x4c, 0x00);

    gpio_pin_configure_dt(cfg->adc_drdy, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(cfg->adc_drdy, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&cfg->adc_cb, fourwire_trigger, BIT(cfg->adc_drdy->pin));
    gpio_add_callback(cfg->adc_drdy->port, &cfg->adc_cb);

    // private state
    cfg->num_mvg_avg  = 10;
    cfg->mvg_avg_vals = calloc(cfg->num_mvg_avg, sizeof(double));
    cfg->mvg_avg_nxt  = -1;

    k_thread_create(&fourwire_thread_data, fourwire_stack_area,
                    K_THREAD_STACK_SIZEOF(fourwire_stack_area),
                    fourwire_process, NULL, NULL, NULL,
                    3, 0, K_NO_WAIT);
    k_thread_start(&fourwire_thread_data);

    // start readings
    cmd = 0x08; // START/SYNC
    i2c_write(cfg->adc_i2c, &cmd, 1, cfg->adc_addr);
}

double fourwire_read(fourwire_config_t *cfg) {
    return cfg->resistance;
}

void fourwire_set_ref(fourwire_config_t *cfg, int idx) {
    for (int i = 0; i < cfg->num_mux_pins; i++) {
        gpio_pin_set_dt(&cfg->mux_pins[i], idx & (1 << i));
    }
}

int32_t fourwire_read_adc(fourwire_config_t *cfg) {
    uint8_t data[3];
    i2c_burst_read(cfg->adc_i2c, cfg->adc_addr, 0x10, data, 3); // RDATA

    int32_t val = (data[0] << 16) | (data[1] << 8) | data[2];
    if (data[0] & 0x80) {
        val |= 0xff000000;
    }

    return val;
}

void fourwire_trigger(const struct device *port, 
                      struct gpio_callback *cb, gpio_port_pins_t pins) {
    fourwire_config_t *cfg = CONTAINER_OF(cb, fourwire_config_t, adc_cb);
    while (k_msgq_put(&fourwire_msgq, &cfg, K_NO_WAIT) != 0) {
        k_msgq_purge(&fourwire_msgq);
    }
}

void fourwire_process(void* p1, void* p2, void* p3) {
    // TODO PGA?
    // TODO offset calibration?

    fourwire_config_t *cfg;
    double res;
    int32_t val;

    while (1) {
        k_msgq_get(&fourwire_msgq, &cfg, K_FOREVER); // wait!

        val = fourwire_read_adc(cfg);

        if (val == 0x7fffff) { // resistance too big
            res = INFINITY;
            if (cfg->best_idx < cfg->num_ref_vals - 1) {
                cfg->best_idx++;
                cfg->mvg_avg_nxt = -1;
            }
        } else { // works, but can do better with smaller ref?
            res = val * cfg->ref_vals[cfg->best_idx] / 0x7fffff;
            if (cfg->best_idx > 0 && res < 0.9 * cfg->ref_vals[cfg->best_idx - 1]) {
                cfg->best_idx--;
                cfg->mvg_avg_nxt = -1;
            }

            // moving average computations
            if (cfg->mvg_avg_nxt == -1) {
                for (int i = 0; i < cfg->num_mvg_avg; i++) {
                    cfg->mvg_avg_vals[i] = res;
                }
                cfg->mvg_avg_nxt = 0;
            } else {
                cfg->mvg_avg_vals[cfg->mvg_avg_nxt++] = res;
                cfg->mvg_avg_nxt %= cfg->num_mvg_avg;

                res = 0;
                for (int i = 0; i < cfg->num_mvg_avg; i++) {
                    res += cfg->mvg_avg_vals[i];
                }
                res /= cfg->num_mvg_avg;
            }
        }

        fourwire_set_ref(cfg, cfg->best_idx);
        cfg->resistance = res;
    }
}
