#define DT_DRV_COMPAT wch_ch32_usbfs

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_ch32_usbfs, CONFIG_USB_DRIVER_LOG_LEVEL);

// Zephyr API implementation
int usb_dc_attach(void) {
    return -ENOTSUP;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb) {
    return -ENOTSUP;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb) {
    ARG_UNUSED(cb);
}

int usb_dc_set_address(const uint8_t addr) {
    return -ENOTSUP;
}

int usb_dc_ep_start_read(uint8_t ep, size_t len) {
    return -ENOTSUP;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg) {
    return -ENOTSUP;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg) {
    return -ENOTSUP;
}

int usb_dc_ep_set_stall(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_clear_stall(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled) {
    return -ENOTSUP;
}

int usb_dc_ep_enable(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_disable(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
        const uint32_t data_len, uint32_t *const ret_bytes) {
    return -ENOTSUP;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data,
        uint32_t max_data_len, uint32_t *read_bytes) {
    return -ENOTSUP;
}

int usb_dc_ep_read_continue(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data,
        const uint32_t max_data_len, uint32_t *const read_bytes) {
    return -ENOTSUP;
}

int usb_dc_ep_halt(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_flush(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_ep_mps(const uint8_t ep) {
    return -ENOTSUP;
}

int usb_dc_detach(void) {
    return -ENOTSUP;
}

int usb_dc_reset(void) {
    return -ENOTSUP;
}

int usb_dc_wakeup_request(void) {
    return -ENOTSUP;
}

static int usb_dc_ch32_usbfs_init(void) {
    LOG_ERR("driver not implemented"); // TODO implement
    return 0;
}

SYS_INIT(usb_dc_ch32_usbfs_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
