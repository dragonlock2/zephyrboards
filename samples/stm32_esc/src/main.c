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
        LOG_DBG("bemf %d %d %d",
            sense_read(SENSE_CHANNEL_BEMF_A),
            sense_read(SENSE_CHANNEL_BEMF_B),
            sense_read(SENSE_CHANNEL_BEMF_C)
        );

        LOG_DBG("isense %d %d %d",
            sense_read(SENSE_CHANNEL_ISENSE_A),
            sense_read(SENSE_CHANNEL_ISENSE_B),
            sense_read(SENSE_CHANNEL_ISENSE_C)
        );

        LOG_DBG("vbus %d",
            sense_read(SENSE_CHANNEL_VBUS)
        );

        static rgb_color_E c = RGB_COLOR_OFF;
        rgb_write(c);
        c++;
        if (c == RGB_COLOR_COUNT) { c = RGB_COLOR_OFF; }
        k_msleep(500);
    }
}
