#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "rgb.h"

static const struct gpio_dt_spec leds[3] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
};

static void rgb_write_raw(uint8_t r, uint8_t g, uint8_t b) {
    // TODO PWM support
    gpio_pin_set_dt(&leds[0], r);
    gpio_pin_set_dt(&leds[1], g);
    gpio_pin_set_dt(&leds[2], b);
}

void rgb_init(void) {
    gpio_pin_configure_dt(&leds[0], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&leds[1], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&leds[2], GPIO_OUTPUT_INACTIVE);
}

void rgb_write(rgb_color c) {
    switch (c) {
        default:
        case rgb_color::OFF:     rgb_write_raw(0,   0,   0);   break;
        case rgb_color::RED:     rgb_write_raw(255, 0,   0);   break;
        case rgb_color::GREEN:   rgb_write_raw(0,   255, 0);   break;
        case rgb_color::BLUE:    rgb_write_raw(0,   0,   255); break;
        case rgb_color::CYAN:    rgb_write_raw(0,   255, 255); break;
        case rgb_color::MAGENTA: rgb_write_raw(255, 0,   255); break;
        case rgb_color::YELLOW:  rgb_write_raw(255, 255, 0);   break;
        case rgb_color::WHITE:   rgb_write_raw(255, 255, 255); break;
    }
}
