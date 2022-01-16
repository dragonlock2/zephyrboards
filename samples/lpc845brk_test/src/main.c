#include <zephyr.h>

#include <drivers/gpio.h>

const struct gpio_dt_spec led[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios)
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);
struct gpio_callback btn_cb_data;

void btn_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    printk("Button pressed!\r\n");
}

int main() {
    gpio_pin_configure_dt(&led[0], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led[1], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led[2], GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb_data, btn_cb, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb_data);

    int idx = 0;
    while (1) {
        gpio_pin_toggle_dt(&led[idx++]); idx %= 3;
        printk("hi! %d %d\r\n", idx, gpio_pin_get_dt(&btn));
        k_msleep(1000);
    }
}
