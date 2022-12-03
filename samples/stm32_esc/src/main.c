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
    rgb_color_E c = 0;
    while (1) {
        motor_write(0.1, true);
        LOG_DBG("cycle time us: %d", motor_cycle_time_us());

        rgb_write(c);
        c = c == RGB_COLOR_COUNT - 1 ? 0 : c + 1;
        k_msleep(500);
        // TODO feedback over LIN
    }
}
