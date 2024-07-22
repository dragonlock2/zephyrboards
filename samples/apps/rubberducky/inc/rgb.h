#pragma once

enum class rgb_color {
    OFF,
    RED,
    GREEN,
    BLUE,
    CYAN,
    MAGENTA,
    YELLOW,
    WHITE,
};

void rgb_init(void);
void rgb_write(rgb_color c);
