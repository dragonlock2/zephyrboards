#include <zephyr/kernel.h>
#include <zephyrboards/drivers/lin.h>
#include "rgb.h"
#include "sense.h"
#include "motor.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *lin = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

LIN_MSGQ_DEFINE(reqs, 10);

void tx_cb(const struct device *dev, int error, void *user_data) {}

int main() {
    rgb_init();
    sense_init();
    motor_init();

    lin_set_mode(lin, LIN_MODE_RESPONDER);
    lin_set_bitrate(lin, 19200);

    struct zlin_filter rx_filt = {
        .checksum_type = LIN_CHECKSUM_ENHANCED,
        .data_len = 2,
        .id = 0x00,
    };
    lin_add_rx_filter_msgq(lin, &reqs, &rx_filt);

    LOG_DBG("booted!");
    motor_write(0.1, true); // default
    rgb_color_E c = 0;
    while (1) {
        // should be handling in interrupt, just lazy :P
        struct zlin_frame msg;
        if (k_msgq_get(&reqs, &msg, K_NO_WAIT) == 0) {
            motor_write(msg.data[0] / 255.0, msg.data[1] & 0x01);
        }

        uint32_t t = motor_cycle_time_us();
        msg.id = 0x01;
        msg.checksum_type = LIN_CHECKSUM_ENHANCED;
        msg.data_len = 4;
        msg.data[0] = (t >> 24) & 0xFF; // MSB first
        msg.data[1] = (t >> 16) & 0xFF;
        msg.data[2] = (t >> 8)  & 0xFF;
        msg.data[3] = (t >> 0)  & 0xFF;
        lin_send(lin, &msg, K_NO_WAIT, tx_cb, NULL);

        rgb_write(c);
        c = c == RGB_COLOR_COUNT - 1 ? 0 : c + 1;
        k_msleep(50);
    }
}
