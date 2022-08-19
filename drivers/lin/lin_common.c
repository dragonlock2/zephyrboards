#include <zephyrboards/drivers/lin.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lin_common, CONFIG_LIN_LOG_LEVEL);

static void lin_msgq_put(const struct device *dev, struct zlin_frame *frame, void *user_data) {
    struct k_msgq *msgq = (struct k_msgq *)user_data;
    ARG_UNUSED(dev);
    __ASSERT_NO_MSG(msgq);
    if (k_msgq_put(msgq, frame, K_NO_WAIT)) {
        LOG_ERR("Msgq %p overflowed. Frame ID: 0x%x", msgq, frame->id);
    }
}

int z_impl_lin_add_rx_filter_msgq(const struct device *dev, struct k_msgq *msgq,
                                  const struct zlin_filter *filter) {
    const struct lin_driver_api *api = dev->api;
    return api->add_rx_filter(dev, lin_msgq_put, msgq, filter);
}
