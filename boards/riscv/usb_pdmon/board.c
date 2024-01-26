#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb_c/usbc.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_pdmon, CONFIG_USBC_LOG_LEVEL);

/*
 * Zephyr USB PD stack works well, but trying to change the PDO at runtime
 * using usbc_request(port, REQUEST_PE_GET_SRC_CAPS) keeps causing some kind
 * of corruption that eventually leads the host to power cycle the board.
 * For now, we'll just build our own very basic sink stack that'll probably
 * break on some devices.
 */

// private defines
#define USB_PDMON_STACK_SIZE (1024)

static void usb_pdmon_thread(void *, void *, void *);

// private data
K_THREAD_DEFINE(
    usb_pdmon_tid, USB_PDMON_STACK_SIZE,
    usb_pdmon_thread, NULL, NULL, NULL,
    K_PRIO_COOP(1), 0, 0
);

static struct {
    const struct device *tcpc;
    const struct device *vbus;
    struct pd_msg tx_msg, rx_msg;
    uint8_t msg_id;
    uint32_t caps[PDO_MAX_DATA_OBJECTS];
    size_t num_caps;
    bool rejected;
} data;

// private helpers
static void usb_pdmon_alert_handler(const struct device *, void *, enum tcpc_alert alert) {
    switch (alert) {
        case TCPC_ALERT_MSG_STATUS:
            k_thread_resume(usb_pdmon_tid);
            break;

        case TCPC_ALERT_TRANSMIT_MSG_SUCCESS:
            data.msg_id++;
            break;

        default:
            break;
    }
}

static uint32_t usb_pdmon_get_rdo(void) {
    // TODO base on request, else safe default
    for (size_t i = 0; i < data.num_caps; i++) {
        LOG_WRN("pdo%d 0x%x", i + 1, data.caps[i]);
    }
    union pd_rdo rdo;
    rdo.fixed.min_or_max_operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(100);
    rdo.fixed.operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(100);
    rdo.fixed.unchunked_ext_msg_supported = 0;
    rdo.fixed.no_usb_suspend = 1;
    rdo.fixed.usb_comm_capable = 0;
    rdo.fixed.cap_mismatch = 0;
    rdo.fixed.giveback = 0;
    rdo.fixed.object_pos = 1; // chooses pdo
    return rdo.raw_value;
}

static void usb_pdmon_send(void) {
    data.tx_msg.header.port_data_role = TC_ROLE_UFP;
    data.tx_msg.header.specification_revision = PD_REV30;
    data.tx_msg.header.port_power_role = TC_ROLE_SINK;
    data.tx_msg.header.message_id = data.msg_id;
    data.tx_msg.header.extended = 0;
    data.tx_msg.len = PD_CONVERT_PD_HEADER_COUNT_TO_BYTES(data.tx_msg.header.number_of_data_objects);
    if (tcpc_transmit_data(data.tcpc, &data.tx_msg) != 0) {
        LOG_ERR("transmit fail");
    }
}

static void usb_pdmon_thread(void *, void *, void *) {
    data.tcpc = DEVICE_DT_GET(DT_NODELABEL(usbpd));
    data.vbus = DEVICE_DT_GET(DT_NODELABEL(vbus1));
    data.msg_id = 0;

    usbc_vbus_enable(data.vbus, true);
    tcpc_init(data.tcpc);
    tcpc_set_cc(data.tcpc, TC_CC_RD);
    tcpc_set_roles(data.tcpc, TC_ROLE_SINK, TC_ROLE_UFP);
    tcpc_set_alert_handler_cb(data.tcpc, usb_pdmon_alert_handler, NULL);

    bool connected = false;
    while (!connected) {
        // should debounce this tbh...
        enum tc_cc_voltage_state cc1, cc2;
        tcpc_get_cc(data.tcpc, &cc1, &cc2);
        if (cc1 == TC_CC_VOLT_RP_DEF || cc1 == TC_CC_VOLT_RP_1A5 || cc1 == TC_CC_VOLT_RP_3A0) {
            LOG_INF("CC1 connected");
            tcpc_set_cc_polarity(data.tcpc, TC_POLARITY_CC1);
            connected = true;
        } else if (cc2 == TC_CC_VOLT_RP_DEF || cc2 == TC_CC_VOLT_RP_1A5 || cc2 == TC_CC_VOLT_RP_3A0) {
            LOG_INF("CC2 connected");
            tcpc_set_cc_polarity(data.tcpc, TC_POLARITY_CC2);
            connected = true;
        }
        k_msleep(25);
    }

    while (true) {
        if (tcpc_receive_data(data.tcpc, &data.rx_msg) >= 0 && data.rx_msg.type == PD_PACKET_SOP) {
            if (data.rx_msg.len == 0) {
                switch (data.rx_msg.header.message_type) {
                    case PD_CTRL_ACCEPT:
                        LOG_INF("rdo accepted");
                        break;

                    case PD_CTRL_REJECT:
                        data.rejected = true;
                        LOG_WRN("rdo rejected"); // TODO send another request
                        break;

                    case PD_CTRL_PS_RDY:
                        LOG_INF("ps ready");
                        break;

                    default:
                        break;
                }
            } else {
                switch (data.rx_msg.header.message_type) {
                    case PD_DATA_SOURCE_CAP: {
                        data.num_caps = MIN(data.rx_msg.header.number_of_data_objects, PDO_MAX_DATA_OBJECTS);
                        for (size_t i = 0; i < data.num_caps; i++) {
                            uint32_t *caps = (uint32_t*) data.rx_msg.data; // works bc little-endian!
                            data.caps[i] = caps[i];
                        }
                        data.tx_msg.type = PD_PACKET_SOP;
                        data.tx_msg.header.message_type = PD_DATA_REQUEST;
                        data.tx_msg.header.number_of_data_objects = 1;
                        *(uint32_t*) &data.tx_msg.data[0] = usb_pdmon_get_rdo();
                        usb_pdmon_send();
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        k_thread_suspend(usb_pdmon_tid);
    }
}
