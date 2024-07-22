#pragma once

enum hid_os {
    UNKNOWN,
    MACOS,
    WINDOWS,
    LINUX,
};

enum hid_mouse_button {
    NONE = 0x00,
    LEFT = 0x01,
    RIGHT = 0x02,
    MIDDLE = 0x04,
};

struct hid_keyboard_report { // see usb/class/hid.h
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
};

struct hid_mouse_report {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t scroll;
};

void hid_init(void);
hid_os hid_get_os(void);
void hid_keyboard_raw(const hid_keyboard_report &report);
void hid_mouse_raw(const hid_mouse_report &report);
