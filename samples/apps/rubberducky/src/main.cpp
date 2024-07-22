#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usb_hid.h>
#include "error.h"
#include "hid.h"
#include "payload.h"
#include "rgb.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void) {
    rgb_init();
    payload_init();
    hid_init();

    LOG_INF("running payload");
    payload_run();
}
