#define DT_DRV_COMPAT wch_ch32_usbfs

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_ch32_usbfs, CONFIG_USB_DRIVER_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) != 1
#error "driver only supports one instance"
#endif

// register fields copied from <ch32x035_usbfs_device.h>
#define USB_IOEN     (0x00000080)
#define USB_PHY_V33  (0x00000040)
#define UDP_PUE_MASK (0x0000000C)
#define UDP_PUE_10K  (0x00000008)
#define UDP_PUE_1K5  (0x0000000C)
#define UDM_PUE_MASK (0x00000003)

#define USB_IRQ     DT_INST_PROP_BY_IDX(0, irq, 0)
#define USB_EP_MAX  DT_INST_PROP(0, num_bidir_endpoints)
#define USB_OUT_IDX (0) // host => device
#define USB_IN_IDX  (1) // device => host

/* private data */
static struct {
    struct {
        bool isochronous;
        uint8_t mps;
        uint8_t rx_len, rx_idx;
        usb_dc_ep_callback cb;
    } ep_state[USB_EP_MAX][2];
    usb_dc_status_callback status_cb;
    uint8_t addr;
    bool ep0_tog;
    bool ep4_tog;
    bool should_set_address;
    bool setup_available;
    bool vdd5v;

    __attribute__((aligned(4))) uint8_t buffer[USB_EP_MAX][2][64]; // 256 bytes wasted :P
    __attribute__((aligned(4))) struct {
        uint8_t ep0[64];
        uint8_t ep4_out[64];
        uint8_t ep4_in[64];
    } ep_buffer;
    uint8_t ep0_rx[64];
} dev_data;

struct cb_msg {
    bool ep_event;
    uint32_t type;
    uint8_t ep;
};

#define USB_DC_THREAD_STACK_SIZE (1024)
K_MSGQ_DEFINE(usb_dc_msgq, sizeof(struct cb_msg), 8, 4);
K_THREAD_STACK_DEFINE(usb_dc_thread_stack, USB_DC_THREAD_STACK_SIZE);
static struct k_thread usb_dc_thread;

/* private helpers */
static volatile uint32_t *EP_DMA(uint8_t ep) {
    switch (ep) {
        case 0: return &USBFSD->UEP0_DMA; break;
        case 1: return &USBFSD->UEP1_DMA; break;
        case 2: return &USBFSD->UEP2_DMA; break;
        case 3: return &USBFSD->UEP3_DMA; break;
        case 5: return &USBFSD->UEP5_DMA; break;
        case 6: return &USBFSD->UEP6_DMA; break;
        case 7: return &USBFSD->UEP7_DMA; break;
    }
    return NULL;
}

static volatile uint16_t *EP_TX_LEN(uint8_t ep) {
    switch (ep) {
        case 0: return &USBFSD->UEP0_TX_LEN; break;
        case 1: return &USBFSD->UEP1_TX_LEN; break;
        case 2: return &USBFSD->UEP2_TX_LEN; break;
        case 3: return &USBFSD->UEP3_TX_LEN; break;
        case 4: return &USBFSD->UEP4_TX_LEN; break;
        case 5: return &USBFSD->UEP5_TX_LEN; break;
        case 6: return &USBFSD->UEP6_TX_LEN; break;
        case 7: return &USBFSD->UEP7_TX_LEN; break;
    }
    return NULL;
}

static volatile uint16_t *EP_CTRL(uint8_t ep) {
    switch (ep) {
        case 0: return &USBFSD->UEP0_CTRL_H; break;
        case 1: return &USBFSD->UEP1_CTRL_H; break;
        case 2: return &USBFSD->UEP2_CTRL_H; break;
        case 3: return &USBFSD->UEP3_CTRL_H; break;
        case 4: return &USBFSD->UEP4_CTRL_H; break;
        case 5: return &USBFSD->UEP5_CTRL_H; break;
        case 6: return &USBFSD->UEP6_CTRL_H; break;
        case 7: return &USBFSD->UEP7_CTRL_H; break;
    }
    return NULL;
}

static void usb_dc_ch32_usbfs_isr(const struct device *dev) {
    ARG_UNUSED(dev);
    struct cb_msg msg = {
        .ep = 0,
        .ep_event = false,
    };
    uint32_t status = USBFSD->INT_FG;
    if (status & USBFS_UIF_TRANSFER) {
        uint8_t ep = USBFSD->INT_ST & 0x0F;
        uint8_t token = (USBFSD->INT_ST >> 4) & 0x03;
        switch (token) {
            case 0b00: // OUT
                *EP_CTRL(ep) = (*EP_CTRL(ep) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
                if (ep == 0) {
                    // some drivers send next packet before requesting our response
                    memcpy(dev_data.ep0_rx, dev_data.ep_buffer.ep0, 64);
                }
                dev_data.ep_state[ep][USB_OUT_IDX].rx_len = USBFSD->RX_LEN;
                dev_data.ep_state[ep][USB_OUT_IDX].rx_idx = 0;
                msg.ep = ep | USB_EP_DIR_OUT;
                msg.type = USB_DC_EP_DATA_OUT;
                msg.ep_event = true;
                k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
                break;

            case 0b10: // IN
                *EP_CTRL(ep) = (*EP_CTRL(ep) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                if (dev_data.should_set_address) {
                    USBFSD->DEV_ADDR = dev_data.addr;
                    dev_data.should_set_address = false;
                }
                if (ep == 4) {
                    dev_data.ep4_tog = !dev_data.ep4_tog;
                }
                msg.ep = ep | USB_EP_DIR_IN;
                msg.type = USB_DC_EP_DATA_IN;
                msg.ep_event = true;
                k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
                break;

            case 0b11: // SETUP
                *EP_CTRL(0) = (*EP_CTRL(0) & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_R_RES_MASK)) |
                    USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_NAK; // clear stalls too in case host requests too fast
                memcpy(dev_data.ep0_rx, dev_data.ep_buffer.ep0, 64); // note see OUT comment
                dev_data.ep0_tog = true;
                dev_data.setup_available = true;
                msg.ep = USB_CONTROL_EP_OUT;
                msg.type = USB_DC_EP_SETUP;
                msg.ep_event = true;
                k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
                break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    } else if (status & USBFS_UIF_BUS_RST) {
        dev_data.ep0_tog = true;
        dev_data.ep_state[0][0].mps = 64;
        dev_data.ep_state[0][1].mps = 64;

        USBFSD->DEV_ADDR = 0x00;
        *EP_CTRL(0) = USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;

        msg.type = USB_DC_RESUME;
        k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (status & USBFS_UIF_SUSPEND) {
        msg.type = USB_DC_SUSPEND;
        k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
    }
}

/* Zephyr API implementation */
int usb_dc_attach(void) {
    // init registers
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL; // reset
    USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN; // DEV_PU_EN is undocumented but crucial...
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
    USBFSD->DEV_ADDR  = 0x00;
    USBFSD->INT_FG    = 0xFF; // clear interrupts
    USBFSD->INT_EN    = USBFS_UIE_SUSPEND | USBFS_UIE_TRANSFER | USBFS_UIE_BUS_RST;

    // setup ep0
    *EP_DMA(0)    = (uint32_t) &dev_data.ep_buffer;
    *EP_TX_LEN(0) = 0;
    *EP_CTRL(0)   = USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;

    // enable other endpoints but NAK everything
    USBFSD->UEP4_1_MOD = 0xCC;
    USBFSD->UEP2_3_MOD = 0xCC;
    USBFSD->UEP567_MOD = 0x3F;

    for (uint8_t ep = 1; ep < USB_EP_MAX; ep++) {
        if (ep != 4) {
            *EP_DMA(ep) = (uint32_t) &dev_data.buffer[ep];
        }
        *EP_TX_LEN(ep) = 0;
        *EP_CTRL(ep)   = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_NAK;
    }

#if IS_ENABLED(CONFIG_SOC_CH32X035)
    if (dev_data.vdd5v) {
        AFIO->CTLR = (AFIO->CTLR & ~UDP_PUE_MASK) | UDP_PUE_10K;
    } else {
        AFIO->CTLR = (AFIO->CTLR & ~UDP_PUE_MASK) | UDP_PUE_1K5;
    }
#else
    #error "update driver to enable D+ pullup for this SoC"
#endif

    return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb) {
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;
    dev_data.ep_state[ep_idx][is_in].cb = cb;
    return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb) {
    dev_data.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr) {
    dev_data.should_set_address = true;
    dev_data.addr = addr;
    return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg) {
    uint8_t ep = USB_EP_GET_IDX(cfg->ep_addr);
    if ((ep != 0 && cfg->ep_type == USB_DC_EP_CONTROL) ||
        (ep >= USB_EP_MAX) ||
        (cfg->ep_mps > 64)) {
        return -ENOTSUP;
    }
    return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg) {
    uint8_t ep = USB_EP_GET_IDX(ep_cfg->ep_addr);
    bool is_in = USB_EP_GET_DIR(ep_cfg->ep_addr) == USB_EP_DIR_IN;

    dev_data.ep_state[ep][is_in].isochronous = ep_cfg->ep_type == USB_DC_EP_ISOCHRONOUS;
    dev_data.ep_state[ep][is_in].mps = ep_cfg->ep_mps;

    if (ep != 0) {
        if (is_in) {
            *EP_TX_LEN(ep) = 0;
            *EP_CTRL(ep) = (*EP_CTRL(ep) & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK)) | USBFS_UEP_T_RES_NAK;
            if (ep == 4) {
                dev_data.ep4_tog = false;
            }
        } else {
            if (dev_data.ep_state[ep][is_in].isochronous) {
                *EP_CTRL(ep) = (*EP_CTRL(ep) & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)) | USBFS_UEP_R_RES_NONE;
            } else {
                *EP_CTRL(ep) = (*EP_CTRL(ep) & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)) | USBFS_UEP_R_RES_ACK;
            }
        }
    }
    return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep) {
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;
    if (is_in) {
        *EP_CTRL(ep_idx) = (*EP_CTRL(ep_idx) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
    } else {
        *EP_CTRL(ep_idx) = (*EP_CTRL(ep_idx) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_STALL;
    }
    return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep) {
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;
    if (is_in) {
        *EP_CTRL(ep_idx) = (*EP_CTRL(ep_idx) & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK)) | USBFS_UEP_T_RES_NAK;
    } else {
        *EP_CTRL(ep_idx) = (*EP_CTRL(ep_idx) & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)) | USBFS_UEP_R_RES_ACK;
    }
    return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled) {
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;
    if (is_in) {
        *stalled = (*EP_CTRL(ep_idx) & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL;
    } else {
        *stalled = (*EP_CTRL(ep_idx) & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL;
    }
    return 0;
}

int usb_dc_ep_enable(const uint8_t ep) {
    return 0; // always ready
}

int usb_dc_ep_disable(const uint8_t ep) {
    LOG_ERR("usb_dc_ep_disable not implemented");
    return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
        const uint32_t data_len, uint32_t *const ret_bytes) {
    if (USB_EP_DIR_IS_OUT(ep)) {
        return -EINVAL;
    }
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    uint32_t write_cnt = MIN(data_len, dev_data.ep_state[ep_idx][USB_IN_IDX].mps);
    if (ep_idx == 0) {
        memcpy(dev_data.ep_buffer.ep0, data, write_cnt);
    } else if (ep_idx == 4) {
        memcpy(dev_data.ep_buffer.ep4_in, data, write_cnt);
    } else {
        memcpy(dev_data.buffer[ep_idx][USB_IN_IDX], data, write_cnt);
    }

    if (ret_bytes == NULL && data_len != write_cnt) {
        LOG_ERR("driver doesn't support writing %d bytes in one call", data_len);
        return -ENOTSUP;
    }

    *EP_TX_LEN(ep_idx) = write_cnt;
    if (ep_idx == 0) {
        *EP_CTRL(0) = (*EP_CTRL(0) & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG))
            | USBFS_UEP_T_RES_ACK | (dev_data.ep0_tog ? USBFS_UEP_T_TOG : 0);
        dev_data.ep0_tog = !dev_data.ep0_tog;
    } else {
        uint16_t expect = dev_data.ep_state[ep_idx][USB_IN_IDX].isochronous ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK;
        if (ep_idx == 4) {
            *EP_CTRL(4) = (*EP_CTRL(4) & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG))
                | expect | (dev_data.ep4_tog ? USBFS_UEP_T_TOG : 0);
        } else {
            *EP_CTRL(ep_idx) = (*EP_CTRL(ep_idx) & ~USBFS_UEP_T_RES_MASK) | expect;
        }
    }
    if (ret_bytes) {
        *ret_bytes = write_cnt;
    }
    return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data,
        uint32_t max_data_len, uint32_t *read_bytes) {
    if (USB_EP_DIR_IS_IN(ep)) {
        return -EINVAL;
    }
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    uint32_t read_cnt;
    if (ep_idx == 0 && dev_data.setup_available) {
        read_cnt = sizeof(struct usb_setup_packet);
    } else {
        read_cnt = dev_data.ep_state[ep_idx][USB_OUT_IDX].rx_len - dev_data.ep_state[ep_idx][USB_OUT_IDX].rx_idx;
    }

    if (data == NULL && max_data_len == 0) {
        return read_cnt;
    }

    read_cnt = MIN(read_cnt, max_data_len);
    if (ep_idx == 0 && dev_data.setup_available) {
        if (read_cnt != 0) { // only consume if actually used
            dev_data.setup_available = false;
        }
        memcpy(data, dev_data.ep0_rx, read_cnt);
    } else {
        uint8_t rx_idx = dev_data.ep_state[ep_idx][USB_OUT_IDX].rx_idx;
        if (ep_idx == 0) {
            memcpy(data, dev_data.ep0_rx + rx_idx, read_cnt);
        } else if (ep_idx == 4) {
            memcpy(data, dev_data.ep_buffer.ep4_out + rx_idx, read_cnt);
        } else {
            memcpy(data, dev_data.buffer[ep_idx][USB_OUT_IDX] + rx_idx, read_cnt);
        }
        dev_data.ep_state[ep_idx][USB_OUT_IDX].rx_idx += read_cnt;
    }
    if (read_bytes) {
        *read_bytes = read_cnt;
    }
    return 0;
}

int usb_dc_ep_read_continue(const uint8_t ep) {
    if (USB_EP_DIR_IS_IN(ep)) {
        return -EINVAL;
    }
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    *EP_CTRL(ep_idx) = (*EP_CTRL(ep_idx) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data,
        const uint32_t max_data_len, uint32_t *const read_bytes) {
    int ret = usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes);
    if (ret == 0) { // if >0, buffer not empty
        ret = usb_dc_ep_read_continue(ep);
    }
    return ret;
}

int usb_dc_ep_halt(const uint8_t ep) {
    LOG_ERR("usb_dc_ep_halt not implemented");
    return 0;
}

int usb_dc_ep_flush(const uint8_t ep) {
    LOG_ERR("usb_dc_ep_flush not implemented");
    return 0;
}

int usb_dc_ep_mps(const uint8_t ep) {
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;
    return dev_data.ep_state[ep_idx][is_in].mps;
}

int usb_dc_detach(void) {
#if IS_ENABLED(CONFIG_SOC_CH32X035)
    AFIO->CTLR = (AFIO->CTLR & ~UDP_PUE_MASK);
#else
    #error "update driver to disable D+ pullup for this SoC"
#endif
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL; // hold reset
    return 0;
}

int usb_dc_reset(void) {
    LOG_ERR("usb_dc_reset not implemented");
    return 0;
}

int usb_dc_wakeup_request(void) {
    LOG_ERR("usb_dc_wakeup_request not implemented");
    return 0;
}

// following usb_dc_rpi_pico.c and deferring callbacks
static void usb_dc_thread_main(void *, void *, void *) {
    struct cb_msg msg;
    while (true) {
        k_msgq_get(&usb_dc_msgq, &msg, K_FOREVER);
        if (msg.ep_event) {
            uint8_t ep = USB_EP_GET_IDX(msg.ep);
            bool is_in = USB_EP_GET_DIR(msg.ep) == USB_EP_DIR_IN;
            if (dev_data.ep_state[ep][is_in].cb) {
                dev_data.ep_state[ep][is_in].cb(msg.ep, msg.type);
            }
        } else {
            if (dev_data.status_cb) {
                dev_data.status_cb(msg.type, NULL);
            }
        }
    }
}

static int usb_dc_ch32_usbfs_init(void) {
#if IS_ENABLED(CONFIG_SOC_CH32X035)
    dev_data.vdd5v = PWR_VDD_SupplyVoltage() == PWR_VDD_5V;
    if (dev_data.vdd5v) {
        AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) | USB_IOEN;
    } else {
        AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK)) | USB_PHY_V33 | USB_IOEN;
    }

    GPIO_InitTypeDef dm_cfg = {
        .GPIO_Pin   = GPIO_Pin_16,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
    };
    GPIO_Init(GPIOC, &dm_cfg);

    GPIO_InitTypeDef dp_cfg = {
        .GPIO_Pin   = GPIO_Pin_17,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_IPU,
    };
    GPIO_Init(GPIOC, &dp_cfg);
#else
    #error "update driver to init USB GPIO for this SoC"
#endif

    IRQ_CONNECT(USB_IRQ, 0, usb_dc_ch32_usbfs_isr, 0, 0);
    irq_enable(USB_IRQ);
    clock_control_on(DT_INST_PROP_BY_IDX(0, clk, 0), DT_INST_PROP_BY_IDX(0, clk, 1));

    k_thread_create(&usb_dc_thread, usb_dc_thread_stack, USB_DC_THREAD_STACK_SIZE,
        usb_dc_thread_main, NULL, NULL, NULL,
        K_PRIO_COOP(2), 0, K_NO_WAIT);
    k_thread_name_set(&usb_dc_thread, "usb_dc_ch32_usbfs");

    return 0;
}

SYS_INIT(usb_dc_ch32_usbfs_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
