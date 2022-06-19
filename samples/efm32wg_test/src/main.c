#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <drivers/uart.h>

const struct gpio_dt_spec led[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios)
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
struct gpio_callback btn_cb_data;

const struct device *spi0 = DEVICE_DT_GET(DT_NODELABEL(usart0));
const struct device *spi1 = DEVICE_DT_GET(DT_NODELABEL(usart1));

const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(leuart0));
const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(leuart1));

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

    /* SPI test TODO */

    /* I2C test */
    for (uint8_t addr = 0; addr < 128; addr++) {
        if (!i2c_write(i2c0, NULL, 0, addr)) {
            printk("I2C0 device found! %d\r\n", addr);
        }
        if (!i2c_write(i2c1, NULL, 0, addr)) {
            printk("I2C1 device found! %d\r\n", addr);
        }
    }

    /* UART test */
    char c;

    uart_poll_out(uart0, 'a');
    k_msleep(10);
    if (uart_poll_in(uart0, &c) < 0) {
        printk("Didn't receive char from uart0\r\n");
    } else {
        printk("Received '%c' from uart0\r\n", c);
    }

    uart_poll_out(uart1, 'a');
    k_msleep(10);
    if (uart_poll_in(uart1, &c) < 0) {
        printk("Didn't receive char from uart1\r\n");
    } else {
        printk("Received '%c' from uart1\r\n", c);
    }

    /* GPIO test */
    int idx = 0;
    while (1) {
        gpio_pin_toggle_dt(&led[idx++]); idx %= 3;

        printk("Hello World! %d\r\n", idx);

        k_msleep(5000);
    }

    return 0;
}
