#include <zephyr.h>

int main() {
    volatile int8_t c = 0;
    while (1) {
        printk("hi! %d\r\n", c++);
        k_msleep(1000);
    }
}
