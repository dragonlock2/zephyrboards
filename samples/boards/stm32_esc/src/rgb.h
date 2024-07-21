#ifndef RGB_H
#define RGB_H

typedef enum {
    RGB_COLOR_OFF = 0,
    RGB_COLOR_RED,
    RGB_COLOR_GREEN,
    RGB_COLOR_BLUE,
    RGB_COLOR_CYAN,
    RGB_COLOR_MAGENTA,
    RGB_COLOR_YELLOW,
    RGB_COLOR_WHITE,
    RGB_COLOR_COUNT,
} rgb_color_E;

void rgb_init();
void rgb_write(rgb_color_E color);

#endif // RGB_H
