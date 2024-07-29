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

    payload_run();
    hid_keyboard_report k{};
    hid_mouse_report m{};
    hid_keyboard_raw(k);
    hid_mouse_raw(m);
    while (true) {
        rgb_write(rgb_color::GREEN);
        k_msleep(50);
        rgb_write(rgb_color::OFF);
        k_msleep(50);
    }
}
