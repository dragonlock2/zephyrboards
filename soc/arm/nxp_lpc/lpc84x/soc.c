#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include <aarch32/cortex_m/exc.h>

static int nxp_lpc84x_init(const struct device *arg) {
    ARG_UNUSED(arg);
    unsigned int old_level;
    old_level = irq_lock();

    NMI_INIT();

    irq_unlock(old_level);
    return 0;
}

SYS_INIT(nxp_lpc84x_init, PRE_KERNEL_1, 0);
