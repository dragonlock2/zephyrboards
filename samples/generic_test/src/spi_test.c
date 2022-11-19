#include "common.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_PROP(TEST_NODE, spi)

#define NUM_SPI DT_PROP_LEN(TEST_NODE, spi)

static const struct device *spis[] = {
    DT_FOREACH_PROP_ELEM(TEST_NODE, spi, ELEM_TO_DEVICE)
};

static int spi_test(const struct device *arg) {
    LOG_INF("starting SPI test");

    struct spi_config cfg = {0};
    cfg.frequency = 250000;
    cfg.operation |= SPI_OP_MODE_MASTER;
    // cfg.operation |= SPI_MODE_CPOL;
    // cfg.operation |= SPI_MODE_CPHA;
    // cfg.operation |= SPI_MODE_LOOP;
    cfg.operation |= SPI_TRANSFER_LSB;
    cfg.operation |= SPI_WORD_SET(8); // bits/word
    cfg.operation |= SPI_LINES_SINGLE;
    // cfg.operation |= SPI_HOLD_ON_CS;
    // cfg.operation |= SPI_LOCK_ON;
    // cfg.operation |= SPI_CS_ACTIVE_HIGH;

    uint8_t tx[4] = {1,2,3,4};
    struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
    struct spi_buf_set tx_buf_set = { .buffers = &tx_buf, .count = 1 };

    uint8_t rx[4] = {0};
    struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
    struct spi_buf_set rx_buf_set = { .buffers = &rx_buf, .count = 1 };

    for (int i = 0; i < NUM_SPI; i++) {
        LOG_INF("transceiving some bytes on SPI %d", i);
        spi_transceive(spis[i], &cfg, &tx_buf_set, &rx_buf_set);
        for (int j = 0; j < sizeof(tx); j++) {
            LOG_INF("sent: 0x%x\trecvd: 0x%x", tx[j], rx[j]);
        }
    }
    return 0;
}

SYS_INIT(spi_test, APPLICATION, 2);

#endif // DT_NODE_HAS_PROP(TEST_NODE, spi)
