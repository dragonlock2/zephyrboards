#include <zephyr/init.h>
#include <fsl_iocon.h>

static int board_init(void) {
    // P0_17 and P0_23 need to be open-drain for LIN transceiver method
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_17, IOCON_HYS_EN | IOCON_OPENDRAIN_EN);
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_23, IOCON_HYS_EN | IOCON_OPENDRAIN_EN);

	return 0;
}

SYS_INIT(board_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
