#define DT_DRV_COMPAT nxp_lpc84x_gpio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <fsl_gpio.h>
#include <fsl_iocon.h>
#include <fsl_pint.h>

/* inspired by nxp,lpc-gpio in gpio_mcux_lpc.c */

// TODO interrupts allocated from one pool for both ports, init should be called before configure

struct gpio_lpc84x_config {
    uint32_t port;
};

struct gpio_lpc84x_data {
    sys_slist_t callbacks;
};

static int gpio_lpc84x_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags) {
    const struct gpio_lpc84x_config *config = dev->config;

    if ((flags & GPIO_INPUT) && (flags & GPIO_OUTPUT)) {
        return -ENOTSUP;
    }

    if (flags & GPIO_SINGLE_ENDED) {
        return -ENOTSUP;
    }

    #define IOCON_GPIO0(n, _) IOCON_INDEX_PIO0_##n
    #define IOCON_GPIO1(n, _) IOCON_INDEX_PIO1_##n
    static const uint8_t IOCON_MAP[2][32] = {
        { LISTIFY(32, IOCON_GPIO0, (,)) },
        { LISTIFY(22, IOCON_GPIO1, (,)) },
    };

    uint32_t modefunc = IOCON_HYS_EN;
    if (flags & GPIO_PULL_UP)   { modefunc |= IOCON_MODE_PULLUP; }
    if (flags & GPIO_PULL_DOWN) { modefunc |= IOCON_MODE_PULLDOWN; }
    IOCON_PinMuxSet(IOCON, IOCON_MAP[config->port][pin], modefunc);

    if (flags & GPIO_OUTPUT_INIT_LOW)  { GPIO_PinWrite(GPIO, config->port, pin, 0); }
    if (flags & GPIO_OUTPUT_INIT_HIGH) { GPIO_PinWrite(GPIO, config->port, pin, 1); }

    if (flags & GPIO_INPUT)  { GPIO->DIRCLR[config->port] = BIT(pin); }
    if (flags & GPIO_OUTPUT) { GPIO->DIRSET[config->port] = BIT(pin); }

    return 0;
}

static int gpio_lpc84x_port_get_raw(const struct device *dev, uint32_t *value) {
    const struct gpio_lpc84x_config *config = dev->config;
    *value = GPIO_PortRead(GPIO, config->port);
    return 0;
}

static int gpio_lpc84x_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value) {
    const struct gpio_lpc84x_config *config = dev->config;
    GPIO_PortMaskedSet(GPIO, config->port, ~mask);
    GPIO_PortMaskedWrite(GPIO, config->port, value);
    GPIO_PortMaskedSet(GPIO, config->port, 0);
    return 0;
}

static int gpio_lpc84x_port_set_bits_raw(const struct device *dev, uint32_t mask) {
    const struct gpio_lpc84x_config *config = dev->config;
    GPIO_PortSet(GPIO, config->port, mask);
    return 0;
}

static int gpio_lpc84x_port_clear_bits_raw(const struct device *dev, uint32_t mask) {
    const struct gpio_lpc84x_config *config = dev->config;
    GPIO_PortClear(GPIO, config->port, mask);
    return 0;
}

static int gpio_lpc84x_port_toggle_bits(const struct device *dev, uint32_t mask) {
    const struct gpio_lpc84x_config *config = dev->config;
    GPIO_PortToggle(GPIO, config->port, mask);
    return 0;
}

static int gpio_lpc84x_init(const struct device *dev) {
    const struct gpio_lpc84x_config *config = dev->config;
    GPIO_PortInit(GPIO, config->port);
    PINT_Init(PINT);
    return 0;
}

static const struct gpio_driver_api gpio_lpc84x_driver_api = {
    .pin_configure           = gpio_lpc84x_pin_configure,
    .port_get_raw            = gpio_lpc84x_port_get_raw,
    .port_set_masked_raw     = gpio_lpc84x_port_set_masked_raw,
    .port_set_bits_raw       = gpio_lpc84x_port_set_bits_raw,
    .port_clear_bits_raw     = gpio_lpc84x_port_clear_bits_raw,
    .port_toggle_bits        = gpio_lpc84x_port_toggle_bits,
    .pin_interrupt_configure = NULL, // gpio_lpc84x_pin_interrupt_configure,
    .manage_callback         = NULL, // gpio_lpc84x_manage_cb,
};

#define GPIO_LPC84X_INIT(n)                                             \
    static const struct gpio_lpc84x_config gpio_lpc84x_##n##_config = { \
        .port = DT_INST_PROP(n, port),                                  \
    };                                                                  \
                                                                        \
    static struct gpio_lpc84x_data gpio_lpc84x_##n##_data;              \
                                                                        \
    DEVICE_DT_INST_DEFINE(n,                                            \
        &gpio_lpc84x_init,                                              \
        NULL,                                                           \
        &gpio_lpc84x_##n##_data,                                        \
        &gpio_lpc84x_##n##_config,                                      \
        PRE_KERNEL_1,                                                   \
        CONFIG_GPIO_INIT_PRIORITY,                                      \
        &gpio_lpc84x_driver_api                                         \
    );

DT_INST_FOREACH_STATUS_OKAY(GPIO_LPC84X_INIT)
