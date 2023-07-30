#include "common.h"
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(network_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_PROP(TEST_NODE, network)

static struct k_sem lock;
static struct net_mgmt_event_callback mgmt_cb;

static void handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
                    struct net_if *iface) {
    if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
        return;
    }

    for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
        char buf[NET_IPV4_ADDR_LEN];
        if (iface->config.ip.ipv4->unicast[i].addr_type != NET_ADDR_DHCP) {
            continue;
        }
        LOG_INF("Got address: %s", 
            net_addr_ntop(AF_INET, &iface->config.ip.ipv4->unicast[i].address.in_addr,
                buf, sizeof(buf)));
    }

    k_sem_give(&lock);
}

static int network_test(void) {
    k_sem_init(&lock, 0, 1);

    LOG_INF("Waiting for an IPV4 address...");
    net_mgmt_init_event_callback(&mgmt_cb, handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&mgmt_cb);
    net_dhcpv4_start(net_if_get_default());

    k_sem_take(&lock, K_FOREVER);

    return 0;
}

SYS_INIT(network_test, APPLICATION, 9);

#endif // DT_PROP(TEST_NODE, usb)

