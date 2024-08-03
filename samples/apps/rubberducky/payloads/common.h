#pragma once

#include <zephyr/kernel.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/usb/class/hid.h>
#include "hid.h"
#include "rgb.h"

static inline void kb_type(uint8_t key, uint8_t mod) {
    // specific hw/sw combos can miss reports if only send one w/o delay
    hid_keyboard_report k{};
    k.keycode[0] = key;
    k.modifier = mod;
    hid_keyboard_raw(k); // press
    hid_keyboard_raw(k);
    hid_keyboard_raw(k);
    hid_keyboard_raw(k);
    k.keycode[0] = 0;
    k.modifier = 0;
    hid_keyboard_raw(k); // release
    hid_keyboard_raw(k);
    hid_keyboard_raw(k);
    hid_keyboard_raw(k);
}

static inline void kb_print(const char *s) {
    const uint8_t ASCII_TO_CODE[95][2] = {
        {0, HID_KEY_SPACE}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_1},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_APOSTROPHE}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_3},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_4}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_5},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_7}, {0, HID_KEY_APOSTROPHE},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_9}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_0},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_8}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_EQUAL},
        {0, HID_KEY_COMMA}, {0, HID_KEY_MINUS}, {0, HID_KEY_DOT}, {0, HID_KEY_SLASH},
        {0, HID_KEY_0}, {0, HID_KEY_1}, {0, HID_KEY_2}, {0, HID_KEY_3}, {0, HID_KEY_4},
        {0, HID_KEY_5}, {0, HID_KEY_6}, {0, HID_KEY_7}, {0, HID_KEY_8}, {0, HID_KEY_9},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_SEMICOLON}, {0, HID_KEY_SEMICOLON},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_COMMA}, {0, HID_KEY_EQUAL},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_DOT}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_SLASH},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_2},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_A}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_B},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_C}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_D},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_E}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_F},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_G}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_H},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_I}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_J},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_K}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_L},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_M}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_N},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_O}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_P},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_Q}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_R},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_S}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_T},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_U}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_V},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_W}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_X},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_Y}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_Z},
        {0, HID_KEY_LEFTBRACE}, {0, HID_KEY_BACKSLASH}, {0, HID_KEY_RIGHTBRACE},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_6}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_MINUS}, {0, HID_KEY_GRAVE},
        {0, HID_KEY_A}, {0, HID_KEY_B}, {0, HID_KEY_C}, {0, HID_KEY_D}, {0, HID_KEY_E}, {0, HID_KEY_F},
        {0, HID_KEY_G}, {0, HID_KEY_H}, {0, HID_KEY_I}, {0, HID_KEY_J}, {0, HID_KEY_K}, {0, HID_KEY_L},
        {0, HID_KEY_M}, {0, HID_KEY_N}, {0, HID_KEY_O}, {0, HID_KEY_P}, {0, HID_KEY_Q}, {0, HID_KEY_R},
        {0, HID_KEY_S}, {0, HID_KEY_T}, {0, HID_KEY_U}, {0, HID_KEY_V}, {0, HID_KEY_W}, {0, HID_KEY_X},
        {0, HID_KEY_Y}, {0, HID_KEY_Z},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_LEFTBRACE}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_BACKSLASH},
        {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_RIGHTBRACE}, {HID_KBD_MODIFIER_LEFT_SHIFT, HID_KEY_GRAVE},
    };
    while (*s) {
        uint8_t c = *s;
        if (c >= 32 && c <= 126) {
            c -= 32;
            kb_type(ASCII_TO_CODE[c][1], ASCII_TO_CODE[c][0]);
        }
        s++;
    }
}
