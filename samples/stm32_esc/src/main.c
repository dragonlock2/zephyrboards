#include <zephyr/kernel.h>
#include <zephyrboards/drivers/lin.h>
#include "rgb.h"
#include "sense.h"
#include "motor.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *lin = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

int main() {
    lin_set_mode(lin, LIN_MODE_RESPONDER);
    lin_set_bitrate(lin, 19200);

    rgb_init();
    sense_init();
    motor_init();

    LOG_DBG("booted!");
    while (1) {
        rgb_write(RGB_COLOR_RED);
        k_msleep(500);
        // TODO feedback over LIN
    }
}
