#define DT_DRV_COMPAT nxp_lpc84x_gpio

/* Inspired by nxp,lpc-gpio */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include "gpio_utils.h"
#include "iocon_map_lpc84x.h"

#include <fsl_gpio.h>
#include <fsl_pint.h>
#include <fsl_iocon.h>

#define NO_PINT_INT ((1 << sizeof(pint_pin_int_t)) - 1)

struct lpc84x_gpio_config {
    struct gpio_driver_config common;
    GPIO_Type *gpio_base;
    PINT_Type *pint_base;
    uint32_t port_no;
};

struct lpc84x_gpio_data {
    struct gpio_driver_data common;
    sys_slist_t callbacks;
    pint_pin_int_t pint_id[32];
    uint32_t isr_list[8];
    uint32_t isr_list_idx;
};

static int lpc84x_gpio_configure(const struct device *dev, gpio_pin_t pin,
                                 gpio_flags_t flags) {
    const struct lpc84x_gpio_config *config = dev->config;
    GPIO_Type *gpio_base = config->gpio_base;
    uint32_t port = config->port_no;

    if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
        return -ENOTSUP;
    }

    if ((flags & GPIO_SINGLE_ENDED) != 0) {
        return -ENOTSUP;
    }

    // use iocon to set stuff
    uint8_t iocon_pin = IOCON_MAP[port][pin];

    uint32_t PIN_CFG = IOCON_HYS_EN;
    if (flags & GPIO_PULL_UP) {
        PIN_CFG |= IOCON_MODE_PULLUP;
    } else if (flags & GPIO_PULL_DOWN) {
        PIN_CFG |= IOCON_MODE_PULLDOWN;
    }

    IOCON_PinMuxSet(IOCON, iocon_pin, PIN_CFG);

    /* supports access by pin now,you can add access by port when needed */
    if (flags & GPIO_OUTPUT_INIT_HIGH) {
        gpio_base->SET[port] = BIT(pin);
    }

    if (flags & GPIO_OUTPUT_INIT_LOW) {
        gpio_base->CLR[port] = BIT(pin);
    }

    /* input-0,output-1 */
    WRITE_BIT(gpio_base->DIR[port], pin, flags & GPIO_OUTPUT);

    return 0;
}

static int lpc84x_gpio_port_get_raw(const struct device *dev,
                                    uint32_t *value) {
    const struct lpc84x_gpio_config *config = dev->config;
    GPIO_Type *gpio_base = config->gpio_base;

    *value = gpio_base->PIN[config->port_no];

    return 0;
}

static int lpc84x_gpio_port_set_masked_raw(const struct device *dev,
                                           uint32_t mask,
                                           uint32_t value) {
    const struct lpc84x_gpio_config *config = dev->config;
    GPIO_Type *gpio_base = config->gpio_base;
    uint32_t port = config->port_no;

    /* Writing 0 allows R+W, 1 disables the pin */
    gpio_base->MASK[port] = ~mask;
    gpio_base->MPIN[port] = value;
    /* Enable back the pins, user won't assume pins remain masked*/
    gpio_base->MASK[port] = 0U;

    return 0;
}

static int lpc84x_gpio_port_set_bits_raw(const struct device *dev,
                                         uint32_t mask) {
    const struct lpc84x_gpio_config *config = dev->config;
    GPIO_Type *gpio_base = config->gpio_base;

    gpio_base->SET[config->port_no] = mask;

    return 0;
}

static int lpc84x_gpio_port_clear_bits_raw(const struct device *dev,
                                           uint32_t mask) {
    const struct lpc84x_gpio_config *config = dev->config;
    GPIO_Type *gpio_base = config->gpio_base;

    gpio_base->CLR[config->port_no] = mask;

    return 0;
}

static int lpc84x_gpio_port_toggle_bits(const struct device *dev,
                                        uint32_t mask) {
    const struct lpc84x_gpio_config *config = dev->config;
    GPIO_Type *gpio_base = config->gpio_base;

    gpio_base->NOT[config->port_no] = mask;

    return 0;
}

static void lpc84x_gpio_port_isr(const struct device *dev) {
    const struct lpc84x_gpio_config *config = dev->config;
    struct lpc84x_gpio_data *data = dev->data;
    uint32_t enabled_int;
    uint32_t int_flags;
    uint32_t pin;

    for (pin = 0; pin < 32; pin++) {
        if (data->pint_id[pin] != NO_PINT_INT) {
            int_flags = PINT_PinInterruptGetStatus(
                config->pint_base, data->pint_id[pin]);
            enabled_int = int_flags << pin;

            PINT_PinInterruptClrStatus(config->pint_base,
                           data->pint_id[pin]);

            gpio_fire_callbacks(&data->callbacks, dev, enabled_int);
        }
    }
}

static uint32_t get_free_isr(struct lpc84x_gpio_data *data) {
    uint32_t i;
    uint32_t isr;

    for (i = 0; i < data->isr_list_idx; i++) {
        if (data->isr_list[i] != -1) {
            isr = data->isr_list[i];
            data->isr_list[i] = -1;
            return isr;
        }
    }

    return -EINVAL;
}

/* Function configures INPUTMUX device to route pin interrupts to a certain
 * PINT. PINT no. is unknown, rather it's determined from ISR no.
 */
static uint32_t attach_pin_to_isr(uint32_t port, uint32_t pin, uint32_t isr_no) {
    uint32_t pint_idx;

    pint_idx = isr_no - PIN_INT0_IRQn;

    SYSCON->PINTSEL[pint_idx] = (port << 5) | pin; // from user manual

    return pint_idx;
}

static void lpc84x_gpio_port_isr(const struct device *dev);


static int lpc84x_gpio_pin_interrupt_configure(const struct device *dev,
                                               gpio_pin_t pin,
                                               enum gpio_int_mode mode,
                                               enum gpio_int_trig trig) {
    const struct lpc84x_gpio_config *config = dev->config;
    struct lpc84x_gpio_data *data = dev->data;
    pint_pin_enable_t interruptMode = kPINT_PinIntEnableNone;
    GPIO_Type *gpio_base = config->gpio_base;
    uint32_t port = config->port_no;
    uint32_t isr;
    uint32_t pint_idx;
    static bool pint_inited;

    /* Ensure pin used as interrupt is set as input*/
    if ((mode & GPIO_INT_ENABLE) &&
        ((gpio_base->DIR[port] & BIT(pin)) != 0)) {
        return -ENOTSUP;
    }

    switch (mode) {
    case GPIO_INT_MODE_DISABLED:
        interruptMode = kPINT_PinIntEnableNone;
        break;
    case GPIO_INT_MODE_LEVEL:
        if (trig == GPIO_INT_TRIG_HIGH) {
            interruptMode = kPINT_PinIntEnableHighLevel;
        } else if (trig == GPIO_INT_TRIG_LOW) {
            interruptMode = kPINT_PinIntEnableLowLevel;
        } else {
            return -ENOTSUP;
        }
        break;
    case GPIO_INT_MODE_EDGE:
        if (trig == GPIO_INT_TRIG_HIGH) {
            interruptMode = kPINT_PinIntEnableRiseEdge;
        } else if (trig == GPIO_INT_TRIG_LOW) {
            interruptMode = kPINT_PinIntEnableFallEdge;
        } else {
            interruptMode = kPINT_PinIntEnableBothEdges;
        }
        break;
    default:
        return -ENOTSUP;
    }

    /* First time calling this function routes PIN->PINT->NVIC */
    if (data->pint_id[pin] == NO_PINT_INT) {
        isr = get_free_isr(data);
        if (isr == -EINVAL) {
            /* Didn't find any free interrupt in this port */
            return -EBUSY;
        }
        pint_idx = attach_pin_to_isr(port, pin, isr);
        data->pint_id[pin] = pint_idx;
    }

    if (!pint_inited) {
        PINT_Init(config->pint_base);
        pint_inited = true;
    }
    PINT_PinInterruptConfig(config->pint_base, data->pint_id[pin],
        interruptMode,
        (pint_cb_t)lpc84x_gpio_port_isr);

    return 0;
}

static int lpc84x_gpio_manage_cb(const struct device *port,
                   struct gpio_callback *callback, bool set) {
    struct lpc84x_gpio_data *data = port->data;

    return gpio_manage_callback(&data->callbacks, callback, set);
}

static int lpc84x_gpio_init(const struct device *dev) {
    const struct lpc84x_gpio_config *config = dev->config;
    struct lpc84x_gpio_data *data = dev->data;
    int i;

    // also enables clock
    GPIO_PortInit(config->gpio_base, config->port_no);

    PINT_Init(config->pint_base);

    for (i = 0; i < 32; i++) {
        data->pint_id[i] = NO_PINT_INT;
    }

    data->isr_list_idx = 0;

    return 0;
}

static const struct gpio_driver_api lpc84x_gpio_driver_api = {
    .pin_configure = lpc84x_gpio_configure,
    .port_get_raw = lpc84x_gpio_port_get_raw,
    .port_set_masked_raw = lpc84x_gpio_port_set_masked_raw,
    .port_set_bits_raw = lpc84x_gpio_port_set_bits_raw,
    .port_clear_bits_raw = lpc84x_gpio_port_clear_bits_raw,
    .port_toggle_bits = lpc84x_gpio_port_toggle_bits,
    .pin_interrupt_configure = lpc84x_gpio_pin_interrupt_configure,
    .manage_callback = lpc84x_gpio_manage_cb,
};

#define LPC84X_GPIO_IRQ_CONNECT(n, m)                                         \
    do {                                                                      \
        struct lpc84x_gpio_data *data = dev->data;                            \
        IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq),                            \
                DT_INST_IRQ_BY_IDX(n, m, priority),                           \
                lpc84x_gpio_port_isr, DEVICE_DT_INST_GET(n), 0);              \
        irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                            \
        data->isr_list[data->isr_list_idx++] = DT_INST_IRQ_BY_IDX(n, m, irq); \
    } while (0)

#define LPC84X_GPIO_IRQ(n, m) \
    COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, m), (LPC84X_GPIO_IRQ_CONNECT(n, m)), ())


#define LPC84X_GPIO(n)                                                \
    static int lpc_gpio_init_##n(const struct device *dev);           \
                                                                      \
    static const struct lpc84x_gpio_config lpc84x_gpio_config_##n = { \
        .common = {                                                   \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),      \
        },                                                            \
        .gpio_base = GPIO,                                            \
        .pint_base = PINT,                                            \
        .port_no = DT_INST_PROP(n, port),                             \
    };                                                                \
                                                                      \
    static struct lpc84x_gpio_data lpc84x_gpio_data_##n;              \
                                                                      \
    DEVICE_DT_INST_DEFINE(n, lpc_gpio_init_##n, NULL,                 \
            &lpc84x_gpio_data_##n,                                    \
            &lpc84x_gpio_config_##n, PRE_KERNEL_1,                    \
            CONFIG_GPIO_INIT_PRIORITY,                                \
            &lpc84x_gpio_driver_api);                                 \
                                                                      \
    static int lpc_gpio_init_##n(const struct device *dev)            \
    {                                                                 \
        lpc84x_gpio_init(dev);                                        \
                                                                      \
        LPC84X_GPIO_IRQ(n, 0);                                        \
        LPC84X_GPIO_IRQ(n, 1);                                        \
        LPC84X_GPIO_IRQ(n, 2);                                        \
        LPC84X_GPIO_IRQ(n, 3);                                        \
                                                                      \
        return 0;                                                     \
    }

DT_INST_FOREACH_STATUS_OKAY(LPC84X_GPIO)
