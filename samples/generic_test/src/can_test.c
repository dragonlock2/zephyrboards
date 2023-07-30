#include "common.h"
#include <zephyr/random/rand32.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(can_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_PROP(TEST_NODE, can)

const struct can_filter filter = {
    .id    = 0,
    .mask  = 0,
#ifdef CONFIG_CAN_FD_MODE
    // mcan driver drops either FD or standard packets...
    .flags = CAN_FILTER_RTR | CAN_FILTER_DATA | CAN_FILTER_FDF,
#else
    .flags = CAN_FILTER_RTR | CAN_FILTER_DATA,
#endif // CONFIG_CAN_FD_MODE
};

static void can_test_thread(void* p1, void* p2, void* p3) {
    int idx = (int) p1;
    const struct device *can = (const struct device*) p2;
    struct k_msgq *msgq = (struct k_msgq*) p3;
    LOG_INF("thread started for CAN %d!", idx);
#ifdef CONFIG_CAN_FD_MODE
    can_set_mode(can, CAN_MODE_FD);
#else
    can_set_mode(can, CAN_MODE_NORMAL);
#endif // CONFIG_CAN_FD_MODE
    LOG_INF("dynamically setting bitrate to 1Mbps for CAN %d", idx);
    if (can_set_bitrate(can, 1000000)) {
        LOG_ERR("setting bitrate for CAN %d failed!", idx);
        return;
    }
#ifdef CONFIG_CAN_FD_MODE
    LOG_INF("dynamically setting data bitrate to 2Mbps for CAN %d", idx);
    if (can_set_bitrate_data(can, 2000000)) {
        LOG_ERR("setting data bitrate for CAN %d failed!", idx);
        return;
    }
#endif // CONFIG_CAN_FD_MODE
    can_add_rx_filter_msgq(can, msgq, &filter);
    can_start(can);

    struct can_frame txmsg = {
        .id      = 0x69,
        .dlc     = 0, // set later
#ifdef CONFIG_CAN_FD_MODE
        .flags   = CAN_FRAME_FDF | CAN_FRAME_BRS,
#else
        .flags   = 0,
#endif // CONFIG_CAN_FD_MODE
    };

    while (1) {
        uint32_t rand = sys_rand32_get();
        memcpy(txmsg.data, &rand, sizeof(rand));
        txmsg.dlc = can_bytes_to_dlc(sizeof(rand));
        can_send(can, &txmsg, K_FOREVER, NULL, NULL);
        LOG_INF("sent 0x%x on CAN %d", rand, idx);

        struct can_frame rxmsg;
        k_msgq_get(msgq, &rxmsg, K_FOREVER);
        memcpy(&rand, rxmsg.data, sizeof(rand));
        LOG_INF("recvd 0x%x from id 0x%x w/ dlc %d on CAN %d",
                rand, rxmsg.id, rxmsg.dlc, idx);

        k_msleep(1000);
    }
}

#define CREATE_CAN_TEST(node_id, prop, idx)                                           \
    CAN_MSGQ_DEFINE(msgq##idx, 32);                                                   \
    K_THREAD_STACK_DEFINE(can##idx_stack, 1024);                                      \
    struct k_thread can##idx_thread_data;                                             \
                                                                                      \
    static void can_test##idx() {                                                     \
        const struct device *can = DEVICE_DT_GET(DT_PROP_BY_IDX(node_id, prop, idx)); \
        LOG_INF("setting up ping pong thread for CAN %d", idx);                       \
        size_t stack_size = K_THREAD_STACK_SIZEOF(can##idx_stack);                    \
        k_thread_create(&can##idx_thread_data, can##idx_stack, stack_size,            \
                        can_test_thread, idx, (void*) can, &msgq##idx,                \
                        K_PRIO_PREEMPT(0), 0, K_NO_WAIT);                             \
        k_thread_start(&can##idx_thread_data);                                        \
    }
DT_FOREACH_PROP_ELEM(TEST_NODE, can, CREATE_CAN_TEST)

#define RUN_CAN_TEST(node_id, prop, idx) can_test##idx();
static int can_test(void) {
    LOG_INF("starting CAN test");
    DT_FOREACH_PROP_ELEM(TEST_NODE, can, RUN_CAN_TEST)
    return 0;
}

SYS_INIT(can_test, APPLICATION, 8);

#endif // DT_NODE_HAS_PROP(TEST_NODE, can)
