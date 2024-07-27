#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <error.h>
#include <hid.h>

LOG_MODULE_REGISTER(hid, CONFIG_LOG_DEFAULT_LEVEL);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
static const uint8_t keyboard_report_desc[] = HID_KEYBOARD_REPORT_DESC();
static const uint8_t mouse_report_desc[] = HID_MOUSE_REPORT_DESC(3);
#pragma GCC diagnostic pop

static struct {
    const struct device *keyboard_dev, *mouse_dev;
    struct k_sem keyboard_lock, mouse_lock;
    struct k_sem usb_conn;
    uint8_t cnt_wl, cnt_02, cnt_04, cnt_ff;
    hid_os os;
} data;

extern "C" int usb_handle_bos(struct usb_setup_packet *setup, int32_t*, uint8_t**) {
    if (usb_reqtype_is_to_host(setup) && USB_GET_DESCRIPTOR_TYPE(setup->wValue) == USB_DESC_STRING) {
        data.cnt_wl++;
        switch (setup->wLength) {
            case 0x02: data.cnt_02++; break;
            case 0x04: data.cnt_04++; break;
            case 0xff: data.cnt_ff++; break;
        };
    }
    return -ENOTSUP;
}

static void hid_usb_cb(enum usb_dc_status_code cb_status, const uint8_t *param) {
    if (cb_status == USB_DC_CONFIGURED) {
        k_sem_give(&data.usb_conn);
    }
}

static void hid_keyboard_cb(const struct device *dev) {
    k_sem_give(&data.keyboard_lock);
}

static void hid_mouse_cb(const struct device *dev) {
    k_sem_give(&data.mouse_lock);
}

void hid_init(void) {
    data.os = hid_os::UNKNOWN;
    k_sem_init(&data.usb_conn, 0, 1);
    k_sem_init(&data.keyboard_lock, 1, 1);
    k_sem_init(&data.mouse_lock, 1, 1);

    data.keyboard_dev = device_get_binding("HID_0");
    data.mouse_dev = device_get_binding("HID_1");
    if (data.keyboard_dev == NULL || data.mouse_dev == NULL) {
        LOG_ERR("failed get hid device");
        error_fatal(error_reason::HID);
    }

    static const struct hid_ops keyboard_ops = { .int_in_ready = hid_keyboard_cb, };
    static const struct hid_ops mouse_ops = { .int_in_ready = hid_mouse_cb, };
    usb_hid_register_device(data.keyboard_dev, keyboard_report_desc, sizeof(keyboard_report_desc), &keyboard_ops);
    usb_hid_register_device(data.mouse_dev, mouse_report_desc, sizeof(mouse_report_desc), &mouse_ops);

    if (usb_hid_init(data.keyboard_dev) != 0 || usb_hid_init(data.mouse_dev) != 0) {
        LOG_ERR("failed init hid");
        error_fatal(error_reason::HID);
    }
    if (usb_enable(hid_usb_cb) != 0) {
        LOG_ERR("failed enable usb");
        error_fatal(error_reason::HID);
    }
    k_sem_take(&data.usb_conn, K_FOREVER);
    k_msleep(250); // first report ignored if too soon

    // algorithm inspired by https://github.com/qmk/qmk_firmware/blob/master/quantum/os_detection.c
    // due to OS updates, this may not work forever
    if (data.cnt_wl == data.cnt_ff) {
        data.os = hid_os::LINUX;
    } else if (data.cnt_ff >= 2 && data.cnt_04 >= 1) {
        data.os = hid_os::WINDOWS;
    } else if (data.cnt_04 == 0 && data.cnt_02 >= 1) {
        data.os = hid_os::MACOS;
    } else {
        LOG_WRN("unknown os: %d %d %d %d", data.cnt_02, data.cnt_04, data.cnt_ff, data.cnt_wl);
        data.os = hid_os::UNKNOWN;
    }

    LOG_INF("usb connected %d", data.os);
}

hid_os hid_get_os(void) {
    return data.os;
}

void hid_keyboard_raw(const hid_keyboard_report &report) {
    k_sem_take(&data.keyboard_lock, K_FOREVER);
    if (hid_int_ep_write(data.keyboard_dev, reinterpret_cast<const uint8_t*>(&report), sizeof(hid_keyboard_report), NULL) != 0) {
        LOG_ERR("failed keyboard write");
        error_fatal(error_reason::HID);
    }
}

void hid_mouse_raw(const hid_mouse_report &report) {
    k_sem_take(&data.mouse_lock, K_FOREVER);
    if (hid_int_ep_write(data.mouse_dev, reinterpret_cast<const uint8_t*>(&report), sizeof(hid_mouse_report), NULL) != 0) {
        LOG_ERR("failed mouse write");
        error_fatal(error_reason::HID);
    }
}
