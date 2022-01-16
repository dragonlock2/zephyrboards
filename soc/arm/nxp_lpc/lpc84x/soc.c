#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <linker/sections.h>
#include <arch/cpu.h>

#include <fsl_power.h>
#include <fsl_clock.h>

static void clock_init(void) {
    // just set to 30MHz FRO as main clock for now
    POWER_DisablePD(kPDRUNCFG_PD_FRO_OUT);
    POWER_DisablePD(kPDRUNCFG_PD_FRO);
    CLOCK_SetFroOscFreq(kCLOCK_FroOscOut30M);
    CLOCK_SetFroOutClkSrc(kCLOCK_FroSrcFroOsc);
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);
    CLOCK_SetCoreSysClkDiv(1U);

    // IOCON used everywhere
    CLOCK_EnableClock(kCLOCK_Iocon);

    // switch everything to main clock
    CLOCK_Select(kUART0_Clk_From_MainClk);
}

static int nxp_lpc84x_init(const struct device *arg) {
    ARG_UNUSED(arg);
    unsigned int old_level;
    old_level = irq_lock();

    clock_init();
    NMI_INIT();

    irq_unlock(old_level);
    return 0;
}

SYS_INIT(nxp_lpc84x_init, PRE_KERNEL_1, 0);
