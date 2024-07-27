#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <fsl_clock.h>
#if CONFIG_USB_DC_NXP_EHCI
#include "usb_phy.h"
#include "usb.h"
#endif

static int k66f_usbhs_init(void) {
    // board powered by internal regulator, need to disable inrush limit
    SIM->USBPHYCTL |= SIM_USBPHYCTL_USBDISILIM(1);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usbhs0), okay) && (CONFIG_USB_DC_NXP_EHCI || CONFIG_UDC_NXP_EHCI)
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_UsbPhySrcExt, CONFIG_OSC_XTAL0_FREQ);
    CLOCK_EnableUsbhs0Clock(kCLOCK_UsbPhySrcExt, 0);

#if CONFIG_USB_DC_NXP_EHCI
    usb_phy_config_struct_t usbPhyConfig = {
        .D_CAL     = 0b0111,
        .TXCAL45DP = 0b0110,
        .TXCAL45DM = 0b0110,
    };
    USB_EhciPhyInit(kUSB_ControllerEhci0, CONFIG_OSC_XTAL0_FREQ, &usbPhyConfig);
#endif
#endif

    return 0;
}

SYS_INIT(k66f_usbhs_init, PRE_KERNEL_2, 0);

#ifdef CONFIG_MCUBOOT

static int k66f_usbhs_sd_init(void) {
    static uint8_t fat_fs[1024] __aligned(4); // larger than FATFS
    struct fs_mount_t mp = {
        .type = FS_FATFS,
        .fs_data = &fat_fs,
        .mnt_point = "/SD:",
    };
    fs_mount(&mp); // ignore error
    return 0;
}

SYS_INIT(k66f_usbhs_sd_init, APPLICATION, 0);

#endif
