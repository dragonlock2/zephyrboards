#include <zephyr.h>

#include <device.h>
#include <drivers/clock_control.h>
#include <fsl_clock.h>

#include <drivers/swm.h>
#include <fsl_swm_connections.h>

const struct device* syscon = DEVICE_DT_GET(DT_NODELABEL(syscon));

const struct device* swm = DEVICE_DT_GET(DT_NODELABEL(swm0));

int main() {

    clock_control_on(syscon, (clock_control_subsys_t) kCLOCK_Iocon);

    swm_assign(swm, 
        (swm_pin_t) kSWM_PortPin_P0_0, (swm_function_t) kSWM_USART0_RTS);

    volatile int8_t c = 0;
    while (1) {
        printk("hi! %d\r\n", c++);
        k_msleep(10000);
    }
}
