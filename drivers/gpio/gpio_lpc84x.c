#define DT_DRV_COMPAT nxp_lpc84x_gpio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <fsl_gpio.h>
#include <fsl_iocon.h>
#include <fsl_pint.h>

/* inspired by nxp,lpc-gpio in gpio_mcux_lpc.c */

struct gpio_lpc84x_config {
    uint32_t port;
};

struct gpio_lpc84x_data {
    sys_slist_t callbacks;
};

static struct {
    struct {
        bool valid;
        bool active;
        const struct device *port;
        uint32_t pin;
    } pint[8];
} gpio_lpc84x_shared;

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

static int gpio_lpc84x_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
        enum gpio_int_mode mode, enum gpio_int_trig trig) {
    const struct gpio_lpc84x_config *config = port->config;
    pint_pin_enable_t pint_mode = kPINT_PinIntEnableNone;
    int pint_int = -1;

    switch (mode) {
        case GPIO_INT_MODE_DISABLED:
            pint_mode = kPINT_PinIntEnableNone;
            break;
        
        case GPIO_INT_MODE_LEVEL:
            switch (trig) {
                case GPIO_INT_TRIG_LOW:  pint_mode = kPINT_PinIntEnableLowLevel;  break;
                case GPIO_INT_TRIG_HIGH: pint_mode = kPINT_PinIntEnableHighLevel; break;
                default: return -ENOTSUP; break;
            }
            break;

        case GPIO_INT_MODE_EDGE:
            switch (trig) {
                case GPIO_INT_TRIG_LOW:  pint_mode = kPINT_PinIntEnableFallEdge;  break;
                case GPIO_INT_TRIG_HIGH: pint_mode = kPINT_PinIntEnableRiseEdge;  break;
                case GPIO_INT_TRIG_BOTH: pint_mode = kPINT_PinIntEnableBothEdges; break;
                default: return -ENOTSUP; break;
            }
            break;

        default:
            return -ENOTSUP;
            break;
    }

    for (int i = 0; i < 8; i++) {
        if (gpio_lpc84x_shared.pint[i].valid) {
            if (!gpio_lpc84x_shared.pint[i].active) {
                pint_int = i;
            } else if (gpio_lpc84x_shared.pint[i].port == port &&
                       gpio_lpc84x_shared.pint[i].pin == pin) {
                pint_int = i;
                break; // reuse allocated one if possible
            }
        }
    }
    if (pint_int == -1) {
        return -EBUSY;
    }

    PINT_PinInterruptConfig(PINT, pint_int, pint_mode, NULL);
    gpio_lpc84x_shared.pint[pint_int].active = mode != GPIO_INT_MODE_DISABLED;
    gpio_lpc84x_shared.pint[pint_int].port = port;
    gpio_lpc84x_shared.pint[pint_int].pin = pin;
    SYSCON->PINTSEL[pint_int] = (config->port << 5) | pin; // from user manual

    return 0;
}

static int gpio_lpc84x_manage_callback(const struct device *port, struct gpio_callback *cb, bool set) {
    struct gpio_lpc84x_data *data = port->data;
    return gpio_manage_callback(&data->callbacks, cb, set);
}

static uint32_t gpio_lpc84x_get_pending_int(const struct device *dev) {
    ARG_UNUSED(dev);
    return -ENOTSUP;
}

static void gpio_lpc84x_isr(uint32_t idx) {
    struct gpio_lpc84x_data *data = gpio_lpc84x_shared.pint[idx].port->data;
    uint32_t pins = 1 << gpio_lpc84x_shared.pint[idx].pin;
    PINT_PinInterruptClrStatus(PINT, idx);
    gpio_fire_callbacks(&data->callbacks, gpio_lpc84x_shared.pint[idx].port, pins);
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
    .pin_interrupt_configure = gpio_lpc84x_pin_interrupt_configure,
    .manage_callback         = gpio_lpc84x_manage_callback,
    .get_pending_int         = gpio_lpc84x_get_pending_int,
};

#define GPIO_LPC84X_IRQ_CONNECT(i, m, n)           \
    do {                                           \
        IRQ_CONNECT(n, m, gpio_lpc84x_isr, i, 0);  \
        irq_enable(n);                             \
        gpio_lpc84x_shared.pint[i].valid = true;   \
        gpio_lpc84x_shared.pint[i].active = false; \
    } while (0)

static int gpio_lpc84x_shared_init(void) {
    #define IRQ_PRIO 2 // default

    GPIO_LPC84X_IRQ_CONNECT(0, IRQ_PRIO, PIN_INT0_IRQn);
    GPIO_LPC84X_IRQ_CONNECT(1, IRQ_PRIO, PIN_INT1_IRQn);
    GPIO_LPC84X_IRQ_CONNECT(2, IRQ_PRIO, PIN_INT2_IRQn);
    GPIO_LPC84X_IRQ_CONNECT(3, IRQ_PRIO, PIN_INT3_IRQn);
    GPIO_LPC84X_IRQ_CONNECT(4, IRQ_PRIO, PIN_INT4_IRQn);
    GPIO_LPC84X_IRQ_CONNECT(5, IRQ_PRIO, PIN_INT5_DAC1_IRQn);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart3), disabled)
    GPIO_LPC84X_IRQ_CONNECT(6, IRQ_PRIO, PIN_INT6_USART3_IRQn);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart4), disabled)
    GPIO_LPC84X_IRQ_CONNECT(7, IRQ_PRIO, PIN_INT7_USART4_IRQn);
#endif

    return 0;
}

SYS_INIT(gpio_lpc84x_shared_init, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY);

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
