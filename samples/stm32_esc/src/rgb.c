#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "rgb.h"

/* private constants */
typedef struct {
    bool r, g, b;
} rgb_config_S;

static const rgb_config_S RGB_CONFIGS[] = {
    [RGB_COLOR_OFF]     = {.r = 0, .g = 0, .b = 0},
    [RGB_COLOR_RED]     = {.r = 1, .g = 0, .b = 0},
    [RGB_COLOR_GREEN]   = {.r = 0, .g = 1, .b = 0},
    [RGB_COLOR_BLUE]    = {.r = 0, .g = 0, .b = 1},
    [RGB_COLOR_CYAN]    = {.r = 0, .g = 1, .b = 1},
    [RGB_COLOR_MAGENTA] = {.r = 1, .g = 0, .b = 1},
    [RGB_COLOR_YELLOW]  = {.r = 1, .g = 1, .b = 0},
    [RGB_COLOR_WHITE]   = {.r = 1, .g = 1, .b = 1},
};

static const struct gpio_dt_spec RGB_LEDS[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_red),   gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue),  gpios),
};

/* public functions */
void rgb_init() {
    gpio_pin_configure_dt(&RGB_LEDS[0], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&RGB_LEDS[1], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&RGB_LEDS[2], GPIO_OUTPUT_INACTIVE);
}

void rgb_write(rgb_color_E color) {
    if (color >= RGB_COLOR_COUNT) { return; }
    gpio_pin_set_dt(&RGB_LEDS[0], RGB_CONFIGS[color].r);
    gpio_pin_set_dt(&RGB_LEDS[1], RGB_CONFIGS[color].g);
    gpio_pin_set_dt(&RGB_LEDS[2], RGB_CONFIGS[color].b);
}
