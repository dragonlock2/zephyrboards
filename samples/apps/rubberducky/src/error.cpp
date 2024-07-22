#include <zephyr/kernel.h>
#include <rgb.h>
#include "error.h"

void error_fatal(error_reason code) {
    switch (code) {
        case error_reason::HID:
            for (int i = 0; i < 2; i++) {
                rgb_write(rgb_color::RED);
                k_msleep(50);
                rgb_write(rgb_color::OFF);
                k_msleep(200);
            }
            break;

        case error_reason::PAYLOAD:
            for (int i = 0; i < 3; i++) {
                rgb_write(rgb_color::RED);
                k_msleep(50);
                rgb_write(rgb_color::OFF);
                k_msleep(200);
            }
            break;
    }
    while (1) { k_msleep(1); }
}
