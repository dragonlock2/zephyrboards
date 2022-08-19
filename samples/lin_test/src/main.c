#include <zephyr.h>
#include <zephyrboards/drivers/lin.h>

const struct device *dev = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

int main() {
    printk("Hello World %p\r\n", dev);

    struct k_msgq *msgq;
    struct zlin_filter *filter;
    lin_add_rx_filter_msgq(dev, msgq, filter);
    lin_remove_rx_filter(dev, 0);
    return 0;
}
