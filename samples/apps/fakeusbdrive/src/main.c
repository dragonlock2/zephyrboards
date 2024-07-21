#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>

int main() {
    usb_enable(NULL);

    return 0;
}
