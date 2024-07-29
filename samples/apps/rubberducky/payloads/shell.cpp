#include "common.h"

extern "C" void run(void) {
    switch (hid_get_os()) {
        case hid_os::MACOS: {
            rgb_write(rgb_color::GREEN);
            kb_type(HID_KEY_SPACE, HID_KBD_MODIFIER_LEFT_UI);
            k_msleep(100);
            kb_print("Terminal.app");
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(1500);
            kb_print("echo hello world!"); // TODO replace w/ payload
            kb_type(HID_KEY_ENTER, 0);
            break;
        }
        case hid_os::WINDOWS: {
            rgb_write(rgb_color::BLUE);
            kb_type(HID_KEY_R, HID_KBD_MODIFIER_LEFT_UI);
            k_msleep(100);
            kb_print("powershell.exe");
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(1000);
            kb_print("echo \"hello world!\""); // TODO replace w/ payload
            kb_type(HID_KEY_ENTER, 0);
            break;
        }
        case hid_os::LINUX: { // Ubuntu
            rgb_write(rgb_color::RED);
            kb_type(HID_KEY_T, HID_KBD_MODIFIER_LEFT_CTRL | HID_KBD_MODIFIER_LEFT_ALT);
            k_msleep(1000);
            kb_print("echo hello world!"); // TODO replace w/ payload
            kb_type(HID_KEY_ENTER, 0);
            break;
        }
        default: {
            printk("unknown os");
            while (true) {
                rgb_write(rgb_color::YELLOW);
                k_msleep(100);
                rgb_write(rgb_color::OFF);
                k_msleep(100);
            }
            break;
        }
    }
}

LL_EXTENSION_SYMBOL(run);
