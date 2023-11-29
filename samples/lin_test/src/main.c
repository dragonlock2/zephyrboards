#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyrboards/drivers/lin.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lin0));
static struct k_sem lock;

static void print_msg(const struct lin_frame *msg) {
    LOG_INF("id=0x%x, checksum=%d, len=%d", msg->id, msg->type, msg->len);
    LOG_HEXDUMP_INF(msg->data, msg->len, "data=");
}

static int header_cb(const struct device *dev, struct lin_frame *msg, void *data) {
    switch (msg->id) {
        case 0x00:
            msg->type = LIN_CHECKSUM_ENHANCED;
            msg->len = 2;
            msg->data[0] = 0x69;
            msg->data[1] = 0x42;
            return LIN_ACTION_SEND;
            break;

        case 0x01:
            msg->type = LIN_CHECKSUM_AUTO;
            msg->len = 0;
            return LIN_ACTION_RECEIVE;
            break;
    }
    k_sem_give(&lock);
    return LIN_ACTION_NONE;
}

static void tx_cb(const struct device *dev, int error, void *data) {
    if (error) {
        LOG_ERR("tx error %d", error);
    } else {
        LOG_INF("tx done %p", data);
    }
    k_sem_give(&lock);
}

static void rx_cb(const struct device *dev, int error, const struct lin_frame *msg, void *data) {
    if (error) {
        LOG_ERR("rx error %d", error);
    } else {
        print_msg(msg);
    }
    k_sem_give(&lock);
}

int main() {
    // configure
    k_sem_init(&lock, 0, 1);
    if (lin_set_bitrate(dev, 19200) ||
        lin_set_header_callback(dev, header_cb, NULL) ||
        lin_set_tx_callback(dev, tx_cb, NULL) ||
        lin_set_rx_callback(dev, rx_cb, NULL)) {
        LOG_ERR("failed to configure");
    }

    // commander
    lin_set_mode(dev, LIN_MODE_COMMANDER);

    struct lin_frame msg = {
        .id = 0x00,
        .type = LIN_CHECKSUM_ENHANCED,
        .len = 2,
        .data = {0x69, 0x42},
    };
    lin_send(dev, &msg);
    k_sem_take(&lock, K_FOREVER);

    lin_receive(dev, 0x01, LIN_CHECKSUM_AUTO, 0);
    k_sem_take(&lock, K_FOREVER);

    // responder
    lin_set_mode(dev, LIN_MODE_RESPONDER);
}
