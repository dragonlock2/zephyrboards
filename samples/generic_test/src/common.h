#ifndef COMMON_H
#define COMMON_H

#include <zephyr/device.h>
#include <zephyr/init.h>

#define TEST_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(test)

#define ELEM_TO_GPIO(node_id, prop, idx) \
    GPIO_DT_SPEC_GET(DT_PROP_BY_IDX(node_id, prop, idx), gpios),

#define ELEM_TO_DEVICE(node_id, prop, idx) \
    DEVICE_DT_GET(DT_PROP_BY_IDX(node_id, prop, idx)),

#endif // COMMON_H