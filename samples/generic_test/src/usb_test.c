#include "common.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_PROP(TEST_NODE, usb)

static int usb_test(const struct device *arg) {
    LOG_INF("enabling USB, returned %d, plug in to check", usb_enable(NULL));
    return 0;
}

SYS_INIT(usb_test, APPLICATION, 5);

#endif // DT_PROP(TEST_NODE, usb)
