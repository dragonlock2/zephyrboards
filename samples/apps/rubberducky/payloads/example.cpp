#include "common.h"

extern "C" void run(void) {
    printk("setting rgb to os\r\n");
    switch (hid_get_os()) {
        case hid_os::MACOS:   rgb_write(rgb_color::GREEN); break;
        case hid_os::WINDOWS: rgb_write(rgb_color::BLUE);  break;
        case hid_os::LINUX:   rgb_write(rgb_color::RED);   break;
        default: rgb_write(rgb_color::OFF); break;
    }

    printk("typing stuff\r\n");
    kb_print("hello world!");

    printk("mouse jiggling\r\n");
    while (true) {
        hid_mouse_report m{};
        m.x = 10;
        hid_mouse_raw(m);
        k_msleep(10);
        m.x = -10;
        hid_mouse_raw(m);
        k_msleep(10);
    }
}

LL_EXTENSION_SYMBOL(run);
