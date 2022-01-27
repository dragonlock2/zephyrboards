#include "fourwire.h"

#include <drivers/i2c.h>
#include <math.h>
#include <stdlib.h>

void fourwire_init(fourwire_config_t *cfg) {
    // mux setup
    for (int i = 0; i < cfg->num_mux_pins; i++) {
        gpio_pin_configure_dt(&cfg->mux_pins[i], GPIO_OUTPUT_INACTIVE);
    }

    cfg->curr_ref = cfg->num_ref_vals - 1;
    fourwire_set_ref(cfg, cfg->curr_ref);

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

    cfg->curr_gain = 0; // 1x gain
    fourwire_set_gain(cfg, cfg->curr_gain);

    gpio_pin_configure_dt(cfg->adc_drdy, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(cfg->adc_drdy, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&cfg->adc_cb, fourwire_trigger, BIT(cfg->adc_drdy->pin));
    gpio_add_callback(cfg->adc_drdy->port, &cfg->adc_cb);

    // moving average init
    cfg->num_mvg_avg  = 10;
    cfg->mvg_avg_vals = calloc(cfg->num_mvg_avg, sizeof(double));
    cfg->mvg_avg_nxt  = -1;

    // start process thread
    k_sem_init(&cfg->adc_pending, 0, 1);

    k_thread_create(&cfg->thread_data, cfg->thread_stack, cfg->thread_stack_size,
                    fourwire_process, cfg, NULL, NULL,
                    K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    k_thread_start(&cfg->thread_data);

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

void fourwire_set_gain(fourwire_config_t *cfg, int gain) {
    uint8_t old;
    i2c_reg_read_byte(cfg->adc_i2c, cfg->adc_addr, 0x20, &old);
    old = (old & 0xf1) | (gain << 1);
    i2c_reg_write_byte(cfg->adc_i2c, cfg->adc_addr, 0x40, old);
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
    k_sem_give(&cfg->adc_pending);
}

void fourwire_process(void* p1, void* p2, void* p3) {
    fourwire_config_t *cfg = (fourwire_config_t*) p1;
    int32_t val;
    double res;

    while (1) {
        k_sem_take(&cfg->adc_pending, K_FOREVER);
        val = fourwire_read_adc(cfg);

        if (val == 0x7fffff) {
            res = INFINITY;
            if (cfg->curr_gain != 0) { // wrong gain?
                cfg->curr_gain = 0;
            } else if (cfg->curr_ref < cfg->num_ref_vals - 1) { // wrong ref?
                cfg->curr_ref++;
                cfg->curr_gain = 0;
            }
            cfg->mvg_avg_nxt = -1;
        } else {
            res = val * cfg->ref_vals[cfg->curr_ref] / 
                    (0x7fffff * (1 << cfg->curr_gain));

            if (cfg->curr_ref > 0 && 
                    res < 0.9 * cfg->ref_vals[cfg->curr_ref - 1]) { // better ref?
                cfg->curr_ref--;
                cfg->curr_gain = 0;
                cfg->mvg_avg_nxt = -1;
            } else { // better gain?
                uint32_t gain = (0x7fffff * (1 << cfg->curr_gain)) / val * 0.9;
                if      (gain >= 128) { cfg->curr_gain = 7; }
                else if (gain >= 64)  { cfg->curr_gain = 6; }
                else if (gain >= 32)  { cfg->curr_gain = 5; }
                else if (gain >= 16)  { cfg->curr_gain = 4; }
                else if (gain >= 8)   { cfg->curr_gain = 3; }
                else if (gain >= 4)   { cfg->curr_gain = 2; }
                else if (gain >= 2)   { cfg->curr_gain = 1; }
                else                  { cfg->curr_gain = 0; }
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

        fourwire_set_ref(cfg, cfg->curr_ref);
        fourwire_set_gain(cfg, cfg->curr_gain);
        cfg->resistance = res;
    }
}
