#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <math.h>

#include "fourwire.h"

// devices
const struct device *leds[] = {
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(red_pwm_led))),
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(green_pwm_led))),
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(blue_pwm_led))),
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(user_btn), gpios);

const struct gpio_dt_spec mux_pins[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_a0), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_a1), gpios),
};

const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(sercom4));

const struct gpio_dt_spec drdy = GPIO_DT_SPEC_GET(DT_NODELABEL(drdy), gpios);

// fun RGB stuffs
#define RED_NODE   DT_ALIAS(red_pwm_led)
#define GREEN_NODE DT_ALIAS(green_pwm_led)
#define BLUE_NODE  DT_ALIAS(blue_pwm_led)

#define PWM_PERIOD PWM_MSEC(10)

void set_rgb(double r, double g, double b) {
    // no invert setting :(
    pwm_set(leds[0], DT_PWMS_CHANNEL(RED_NODE),
        PWM_PERIOD, PWM_PERIOD - r * PWM_PERIOD, DT_PWMS_FLAGS(RED_NODE));
    pwm_set(leds[1], DT_PWMS_CHANNEL(GREEN_NODE),
        PWM_PERIOD, PWM_PERIOD - g * PWM_PERIOD, DT_PWMS_FLAGS(GREEN_NODE));
    pwm_set(leds[2], DT_PWMS_CHANNEL(BLUE_NODE),
        PWM_PERIOD, PWM_PERIOD - b * PWM_PERIOD, DT_PWMS_FLAGS(BLUE_NODE));
}

// app
K_THREAD_STACK_DEFINE(fourwire_stack_area, 256);

int main() {
    usb_enable(NULL);

    set_rgb(0.01, 0.01, 0.01);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    const double REF_RES[] = {
        10.0,
        10000.0,
        10000000.0,
    };

    fourwire_config_t fourwire_cfg = {
        .ref_vals = REF_RES,
        .num_ref_vals = 3,
        .mux_pins = mux_pins,
        .num_mux_pins = 2,
        .adc_i2c = i2c,
        .adc_drdy = &drdy,
        .adc_addr = 0x40,

        .thread_stack = fourwire_stack_area,
        .thread_stack_size = K_THREAD_STACK_SIZEOF(fourwire_stack_area),
    };
    fourwire_init(&fourwire_cfg);

    while (1) {
        double val = fourwire_read(&fourwire_cfg);

        if (val < 1.0) {
            printk("%f mΩ\r\n", val * 1000.0);
        } else if (val < 1000.0) {
            printk("%f Ω\r\n", val);
        } else if (val < 1000000.0) {
            printk("%f kΩ\r\n", val / 1000.0);
        } else if (val < INFINITY) {
            printk("%f MΩ\r\n", val / 1000000.0);
        } else {
            printk("%f Ω\r\n", val);
        }
        k_msleep(100);
    }
}
