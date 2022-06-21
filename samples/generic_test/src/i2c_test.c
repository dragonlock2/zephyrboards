#include "common.h"
#include <drivers/i2c.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_PROP(TEST_NODE, i2c)

#define NUM_I2C DT_PROP_LEN(TEST_NODE, i2c)

static const struct device *i2cs[] = {
    DT_FOREACH_PROP_ELEM(TEST_NODE, i2c, ELEM_TO_DEVICE)
};

static int i2c_test(const struct device *arg) {
    LOG_INF("starting I2C test");
    for (int i = 0; i < NUM_I2C; i++) {
        LOG_INF("scanning for devices on I2C %d", i);
        for (uint8_t addr = 0; addr < 128; addr++) {
            if (!i2c_write(i2cs[i], NULL, 0, addr)) {
                LOG_INF("device 0x%x found!", addr);
            }
        }
    }
    return 0;
}

SYS_INIT(i2c_test, APPLICATION, 3);

#endif // DT_NODE_HAS_PROP(TEST_NODE, i2c)
