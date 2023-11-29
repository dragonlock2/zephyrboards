#include <zephyr/kernel.h>
#include <zephyrboards/drivers/lin.h>
#include "rgb.h"
#include "sense.h"
#include "motor.h"

const struct device *lin = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

static int header_cb(const struct device *dev, struct lin_frame *msg, void *data) {
    switch (msg->id) {
        case 0x00:
            msg->type = LIN_CHECKSUM_ENHANCED;
            msg->len = 2;
            return LIN_ACTION_RECEIVE;
            break;

        case 0x01:
            uint32_t t = motor_cycle_time_us();
            msg->type = LIN_CHECKSUM_ENHANCED;
            msg->len = 4;
            msg->data[0] = (t >> 24) & 0xFF; // big-endian
            msg->data[1] = (t >> 16) & 0xFF;
            msg->data[2] = (t >> 8)  & 0xFF;
            msg->data[3] = (t >> 0)  & 0xFF;
            return LIN_ACTION_SEND;
            break;
    }
    return LIN_ACTION_NONE;
}

static void rx_cb(const struct device *dev, int error, const struct lin_frame *msg, void *data) {
    if (!error) {
        switch (msg->id) {
            case 0x00:
                motor_write(msg->data[0] / 255.0, msg->data[1] & 0x01);
                break;
        }
    }
}

int main() {
    rgb_init();
    sense_init();
    motor_init();

    lin_set_mode(lin, LIN_MODE_RESPONDER);
    lin_set_bitrate(lin, 19200);
    lin_set_header_callback(lin, header_cb, NULL);
    lin_set_rx_callback(lin, rx_cb, NULL);

    motor_write(0.1, true); // default
    rgb_color_E c = 0;
    while (1) {
        rgb_write(c);
        c = c == RGB_COLOR_COUNT - 1 ? 0 : c + 1;
        k_msleep(50);
    }
}
