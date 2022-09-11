#include <zephyr.h>
#include <logging/log.h>
#include <zephyrboards/drivers/lin.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *dev = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

LIN_MSGQ_DEFINE(msgq, 32);

void lin_tx_handler(const struct device *dev, int error, void *user_data) {
    LOG_INF("finished tx %p %d", dev, error);
}

void print_msg(struct zlin_frame *msg) {
    LOG_INF("msg = id=0x%x,checksum=%d,data_len=%d",
        msg->id, msg->checksum_type, msg->data_len);
    LOG_HEXDUMP_INF(msg->data, msg->data_len, "data=");
}

int main() {
    if (lin_set_bitrate(dev, 19200)) {
        LOG_ERR("couldn't set bitrate");
    }

    LOG_INF("adding auto length/checksum filter to each id");
    struct zlin_filter filter = {
        .checksum_type = LIN_CHECKSUM_AUTO,
        .data_len = 0,
    };
    for (int i = 0; i < LIN_NUM_ID; i++) {
        filter.id = i;
        if (lin_add_rx_filter_msgq(dev, &msgq, &filter) < 0) {
            LOG_ERR("failed to add filter");            
        }
    }

    LOG_INF("doing things as commander");
    if (lin_set_mode(dev, LIN_MODE_COMMANDER)) {
        LOG_ERR("failed to set commander mode");
    }
    struct zlin_frame msg = {
        .id = 42,
        .checksum_type = LIN_CHECKSUM_ENHANCED,
        .data_len = 2,
        .data = {0x69, 0x42},
    };
    print_msg(&msg);
    if (lin_send(dev, &msg, K_MSEC(10), lin_tx_handler, NULL)) {
        LOG_ERR("failed to send msg in time");
    }
    if (lin_receive(dev, 42, K_MSEC(10), lin_tx_handler, NULL)) {
        LOG_ERR("failed to receive msg in time");
    }
    if (k_msgq_get(&msgq, &msg, K_MSEC(500)) == 0) {
        print_msg(&msg);
    }

    LOG_INF("doing things as responder");
    if (lin_set_mode(dev, LIN_MODE_RESPONDER)) {
        LOG_ERR("failed to set responder mode");
    }
    print_msg(&msg);
    if (lin_send(dev, &msg, K_MSEC(500), lin_tx_handler, NULL)) {
        LOG_ERR("failed to send msg in time");
    }

    LOG_INF("just writing an incrementing counter to ID 42");
    struct zlin_frame msg2 = {
        .id = 42,
        .checksum_type = LIN_CHECKSUM_ENHANCED,
        .data_len = 1,
        .data = {0x00},
    };
    while (true) {
        msg2.data[0]++;
        if (lin_send(dev, &msg2, K_MSEC(50), lin_tx_handler, NULL)) {
            LOG_ERR("failed to put message in queue");
        }
        k_msleep(500);
    }
}
