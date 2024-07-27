#ifndef ZEPHYR_INCLUDE_USB_DEVICE_BOS_DESC_H_
#define ZEPHYR_INCLUDE_USB_DEVICE_BOS_DESC_H_

#include <stdint.h>
#include <zephyr/usb/usb_ch9.h>

#ifdef CONFIG_USB_DEVICE_BOS
#error "os detection workaround not compatabile with CONFIG_USB_DEVICE_BOS"
#endif

int usb_handle_bos(struct usb_setup_packet *setup, int32_t *len, uint8_t **data);

#endif	/* ZEPHYR_INCLUDE_USB_DEVICE_BOS_DESC_H_ */
