#include <zephyr/kernel.h>

int main(void) {
    try {
        throw 69;
    } catch (int &e) {
        printk("caught %d\r\n", e);
    }
    return 0;
}
