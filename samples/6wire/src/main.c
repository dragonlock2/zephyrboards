#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <drivers/i2c.h>
#include <math.h>

// reference resistors in ohms
#define REF_RES_1 10.0
#define REF_RES_2 10000.0
#define REF_RES_3 10000000.0
#define REF_RES_4 INFINITY

// devices
const struct device *leds[] = {
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(red_pwm_led))),
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(green_pwm_led))),
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(blue_pwm_led))),
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(user_btn), gpios);

const struct gpio_dt_spec mux[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_a0), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_a1), gpios),
};

const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(sercom4));

const struct gpio_dt_spec drdy = GPIO_DT_SPEC_GET(DT_NODELABEL(drdy), gpios);

// fun RGB stuffs
#define RED_NODE   DT_ALIAS(red_pwm_led)
#define GREEN_NODE DT_ALIAS(green_pwm_led)
#define BLUE_NODE  DT_ALIAS(blue_pwm_led)

#define PWM_PERIOD 10000 // us

void set_rgb(double r, double g, double b) {
    // no invert setting :(
    pwm_pin_set_usec(leds[0], DT_PWMS_CHANNEL(RED_NODE),
        PWM_PERIOD, PWM_PERIOD - r * PWM_PERIOD, DT_PWMS_FLAGS(RED_NODE));
    pwm_pin_set_usec(leds[1], DT_PWMS_CHANNEL(GREEN_NODE),
        PWM_PERIOD, PWM_PERIOD - g * PWM_PERIOD, DT_PWMS_FLAGS(GREEN_NODE));
    pwm_pin_set_usec(leds[2], DT_PWMS_CHANNEL(BLUE_NODE),
        PWM_PERIOD, PWM_PERIOD - b * PWM_PERIOD, DT_PWMS_FLAGS(BLUE_NODE));
}

// 4wire algorithm

void set_reference(int idx) {
    gpio_pin_set_dt(&mux[0], idx & 0x01);
    gpio_pin_set_dt(&mux[1], idx & 0x02);
}

// app
int main() {
    // 4wire setup
    gpio_pin_configure_dt(&mux[0], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&mux[1], GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    set_reference(2);

    for (uint8_t addr = 0; addr < 128; addr++) {
        if (!i2c_write(i2c, NULL, 0, addr)) {
            printk("I2C device found! %d\r\n", addr);
        }
    }

    while (1) {
        set_rgb(0.01, 0.01, 0.01);

        printk("Hello World! %d\r\n", gpio_pin_get_dt(&btn));
        k_msleep(500);
    }
}