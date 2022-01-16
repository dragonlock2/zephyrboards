#include <zephyr.h>

#include <device.h>
#include <drivers/clock_control.h>
#include <fsl_clock.h>

const struct device* dev = DEVICE_DT_GET(DT_NODELABEL(syscon));

int main() {

    clock_control_on(dev, (clock_control_subsys_t) kCLOCK_Swm);

    volatile int8_t c = 0;
    while (1) {
        printk("hi! %d\r\n", c++);
        k_msleep(10000);
    }
}
