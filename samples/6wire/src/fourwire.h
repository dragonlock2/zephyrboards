#ifndef FOURWIRE_H
#define FOURWIRE_H

#include <zephyr.h>
#include <drivers/gpio.h>

typedef struct {
    const double *ref_vals; // ohms (MUST be in increasing order)
    const size_t num_ref_vals;

    const struct gpio_dt_spec *mux_pins;
    const size_t num_mux_pins;

    const struct device* adc_i2c;
    const struct gpio_dt_spec *adc_drdy;
    const uint8_t adc_addr;

    k_thread_stack_t *thread_stack;
    size_t thread_stack_size;

    // private vars
    int curr_ref;
    int curr_gain; // stored as log2(gain)
    double resistance;

    double *mvg_avg_vals;
    size_t num_mvg_avg;
    int mvg_avg_nxt;

    struct gpio_callback adc_cb;
    struct k_sem adc_pending;
    struct k_thread thread_data;
} fourwire_config_t;

// public functions
void fourwire_init(fourwire_config_t *cfg);
double fourwire_read(fourwire_config_t *cfg);

// "private" helpers
void fourwire_set_ref(fourwire_config_t *cfg, int idx);
void fourwire_set_gain(fourwire_config_t *cfg, int gain);
int32_t fourwire_read_adc(fourwire_config_t *cfg);
void fourwire_trigger(const struct device *port, 
                      struct gpio_callback *cb, gpio_port_pins_t pins);
void fourwire_process(void* p1, void* p2, void* p3);

#endif // FOURWIRE_H
