#include <zephyr/kernel.h>
#include <fsl_clock.h>
#if CONFIG_USB_DC_NXP_EHCI
#include "usb_phy.h"
#include "usb.h"
#endif

static int k66f_usbhs_init(void) {
    // board powered by internal regulator, need to disable inrush limit
    SIM->USBPHYCTL |= SIM_USBPHYCTL_USBDISILIM(1);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usbhs0), okay) && CONFIG_USB_DC_NXP_EHCI
    // TODO remove once use udc driver
    usb_phy_config_struct_t usbPhyConfig = {
        .D_CAL     = 0b0111,
        .TXCAL45DP = 0b0110,
        .TXCAL45DM = 0b0110,
    };
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_UsbPhySrcExt, CONFIG_OSC_XTAL0_FREQ);
    CLOCK_EnableUsbhs0Clock(kCLOCK_UsbPhySrcExt, 0);
    USB_EhciPhyInit(kUSB_ControllerEhci0, CONFIG_OSC_XTAL0_FREQ, &usbPhyConfig);
#endif

    return 0;
}

SYS_INIT(k66f_usbhs_init, PRE_KERNEL_2, 0);
