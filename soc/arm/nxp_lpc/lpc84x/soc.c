#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <fsl_power.h>
#include <soc.h>

static void clock_init(void) {
    // set to 30MHz FRO as main clock
    POWER_DisablePD(kPDRUNCFG_PD_FRO_OUT);
    POWER_DisablePD(kPDRUNCFG_PD_FRO);
    CLOCK_SetFroOscFreq(kCLOCK_FroOscOut30M);
    CLOCK_SetFroOutClkSrc(kCLOCK_FroSrcFroOsc);
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);
    CLOCK_SetCoreSysClkDiv(1);

    // always used
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Swm);

    // use main clock for everything (difficult to do cleanly in devicetree)
    CLOCK_Select(kUART0_Clk_From_MainClk);
    CLOCK_Select(kUART1_Clk_From_MainClk);
    CLOCK_Select(kUART2_Clk_From_MainClk);
    CLOCK_Select(kUART3_Clk_From_MainClk);
    CLOCK_Select(kUART4_Clk_From_MainClk);
}

static int nxp_lpc84x_init(void) {
    clock_init();

    return 0;
}

SYS_INIT(nxp_lpc84x_init, PRE_KERNEL_1, 0);
