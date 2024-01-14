#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>

static void clock_init(void) {
    // TODO set 48MHz clock
}

static int wch_ch32x035_init(void) {
    clock_init();

    return 0;
}

SYS_INIT(wch_ch32x035_init, PRE_KERNEL_1, 0);
