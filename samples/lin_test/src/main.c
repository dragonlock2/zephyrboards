#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyrboards/drivers/lin.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *dev = DEVICE_DT_GET_ANY(zephyrboards_lin_uart);

void print_msg(struct zlin_frame *msg) {
    LOG_INF("id=0x%x, checksum=%d, len=%d", msg->id, msg->type, msg->len);
    LOG_HEXDUMP_INF(msg->data, msg->data_len, "data=");
}

int main() {
    LOG_INF("TODO stuff in commander and responder mode");
}
