#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

const struct gpio_dt_spec led[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios)
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);
struct gpio_callback btn_cb_data;

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

void btn_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    printk("Button pressed!\r\n");
}

int main() {
    printk("clock %d\r\n", CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

    gpio_pin_configure_dt(&led[0], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led[1], GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led[2], GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb_data, btn_cb, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb_data);

    for (uint8_t addr = 0; addr < 128; addr++) {
        if (!i2c_write(i2c_dev, NULL, 0, addr)) {
            printk("I2C device found! %x\r\n", addr);
        }
    }

    const struct device *cdc_dev = device_get_binding("CDC_ACM_0");
    printk("Init USB: %d\r\n", usb_enable(NULL));

    uint8_t i = 0;
    while (1) {
        printk("led %d but %d\r\n", i, gpio_pin_get_dt(&btn));
        gpio_pin_toggle_dt(&led[i++]); i %= 3;
        k_msleep(1000);
    }
}
