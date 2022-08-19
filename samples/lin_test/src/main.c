#include <zephyrboards/drivers/lin.h>
#include <zephyr.h>

int main() {
    const struct device *dev;
    struct k_msgq *msgq;
    struct zlin_filter *filter;
    lin_add_rx_filter_msgq(dev, msgq, filter);
    lin_remove_rx_filter(dev, 0);

    printk("Hello World\r\n");
    return 0;
}
