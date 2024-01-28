#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb_c/usbc.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_pdmon, CONFIG_USBC_LOG_LEVEL);

#ifndef CONFIG_USBC_STACK

/*
 * Zephyr USB PD stack works well, but trying to change the PDO at runtime
 * using usbc_request(port, REQUEST_PE_GET_SRC_CAPS) keeps causing some kind
 * of corruption (might be retries or collisions tbh) that eventually leads
 * the host to power cycle the board. For now, we'll just build our own very
 * basic sink stack that'll probably break on some devices.
 */

// private defines
#define USB_PDMON_STACK_SIZE (1024)

static void usb_pdmon_thread(void *, void *, void *);

typedef struct {
    uint32_t mV, mA;
} usb_pdmon_req_t;

// private data
K_THREAD_DEFINE(
    usb_pdmon_tid, USB_PDMON_STACK_SIZE,
    usb_pdmon_thread, NULL, NULL, NULL,
    K_PRIO_COOP(1), 0, 0
);

K_MSGQ_DEFINE(usb_pdmon_msgq, sizeof(usb_pdmon_req_t), 1, 4);

static struct {
    const struct device *tcpc;
    const struct device *vbus;
    struct pd_msg tx_msg, rx_msg;
    enum pd_rev_type pd_rev;
    uint8_t msg_id;
    uint32_t caps[PDO_MAX_DATA_OBJECTS];
    int num_caps;
    bool rejected, ps_ready;
    usb_pdmon_req_t req, actual;
    struct k_sem actual_lock;
} data;

// private helpers
static void usb_pdmon_alert_handler(const struct device *, void *, enum tcpc_alert alert) {
    switch (alert) {
        case TCPC_ALERT_MSG_STATUS:
            k_wakeup(usb_pdmon_tid);
            break;

        case TCPC_ALERT_TRANSMIT_MSG_SUCCESS:
            data.msg_id++;
            break;

        default:
            break;
    }
}

static uint32_t usb_pdmon_get_rdo(void) {
    // TODO PPS, EPR, AVS support
    k_msgq_get(&usb_pdmon_msgq, &data.req, K_NO_WAIT);
    int idx;
    for (idx = data.num_caps - 1; idx >= 0; idx--) {
        // finds highest voltage less than or equal to requested
        union pd_fixed_supply_pdo_source pdo = { .raw_value = data.caps[idx] };
        if (pdo.type == PDO_FIXED && PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(pdo.voltage) <= data.req.mV) {
            data.req.mV = PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(pdo.voltage);
            data.req.mA = MIN(data.req.mA, PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(pdo.max_current));
            break;
        }
    }
    if (data.rejected || (idx < 0)) {
        data.req.mV = 5000;
        data.req.mA = 100;
        idx = 0;
    }
    LOG_INF("request %dmV %dmA on pdo%d", data.req.mV, data.req.mA, idx);

    union pd_rdo rdo;
    rdo.fixed.min_or_max_operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(data.req.mA);
    rdo.fixed.operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(data.req.mA);
    rdo.fixed.unchunked_ext_msg_supported = 0;
    rdo.fixed.no_usb_suspend = 1;
    rdo.fixed.usb_comm_capable = 1;
    rdo.fixed.cap_mismatch = 0;
    rdo.fixed.giveback = 0;
    rdo.fixed.object_pos = idx + 1; // 1-indexed
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
    data.req.mV = 5000;
    data.req.mA = 3000;
    data.pd_rev = PD_REV30;
    k_sem_init(&data.actual_lock, 1, 1);

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
        k_msleep(100);
    }

    while (true) {
        enum tc_cc_voltage_state cc1, cc2;
        tcpc_get_cc(data.tcpc, &cc1, &cc2);
        if ((data.pd_rev <= PD_REV20) || // non-zero risk of collision
            (cc1 == TC_CC_VOLT_RP_3A0 || cc2 == TC_CC_VOLT_RP_3A0)) { // SinkTxOk added in PD 3.0
            if (data.ps_ready && k_msgq_num_used_get(&usb_pdmon_msgq) != 0) {
                data.tx_msg.type = PD_PACKET_SOP;
                data.tx_msg.header.message_type = PD_CTRL_GET_SOURCE_CAP;
                data.tx_msg.header.number_of_data_objects = 0;
                usb_pdmon_send();
                data.ps_ready = false;
            }
        }
        if (tcpc_receive_data(data.tcpc, &data.rx_msg) >= 0 && data.rx_msg.type == PD_PACKET_SOP) {
            if (data.rx_msg.len == 0) {
                switch (data.rx_msg.header.message_type) {
                    case PD_CTRL_ACCEPT:
                        LOG_INF("rdo accepted");
                        break;

                    case PD_CTRL_REJECT: // host may or may not request again
                        LOG_WRN("rdo rejected");
                        data.rejected = true;
                        data.ps_ready = true;
                        break;

                    case PD_CTRL_PS_RDY:
                        LOG_INF("ps ready");
                        data.rejected = false;
                        data.ps_ready = true;
                        if (k_sem_take(&data.actual_lock, K_MSEC(100))) {
                            LOG_ERR("unable to grab actual lock for writing, dropping");
                        } else {
                            data.actual = data.req;
                            k_sem_give(&data.actual_lock);
                        }
                        break;

                    case PD_CTRL_GOOD_CRC:
                        // ignore
                        break;

                    default:
                        LOG_WRN("ctrl msg %d ignored", data.rx_msg.header.message_type);
                        break;
                }
            } else {
                switch (data.rx_msg.header.message_type) {
                    case PD_DATA_SOURCE_CAP: {
                        data.pd_rev = data.rx_msg.header.specification_revision;
                        data.num_caps = MIN(data.rx_msg.header.number_of_data_objects, PDO_MAX_DATA_OBJECTS);
                        for (size_t i = 0; i < data.num_caps; i++) {
                            uint32_t *caps = (uint32_t*) data.rx_msg.data; // works bc little-endian!
                            data.caps[i] = caps[i];

                            union pd_fixed_supply_pdo_source pdo = { .raw_value = caps[i] };
                            if (pdo.type == PDO_FIXED) {
                                LOG_WRN("fixed pdo%d %dmV %dmA", i,
                                    PD_CONVERT_FIXED_PDO_VOLTAGE_TO_MV(pdo.voltage),
                                    PD_CONVERT_FIXED_PDO_CURRENT_TO_MA(pdo.max_current)
                                );
                            }
                        }
                        data.tx_msg.type = PD_PACKET_SOP;
                        data.tx_msg.header.message_type = PD_DATA_REQUEST;
                        data.tx_msg.header.number_of_data_objects = 1;
                        *(uint32_t*) &data.tx_msg.data[0] = usb_pdmon_get_rdo();
                        usb_pdmon_send();
                        data.ps_ready = false;
                        break;
                    }

                    default:
                        LOG_WRN("data msg %d w/ %d obj ignored", data.rx_msg.header.message_type,
                            data.rx_msg.header.number_of_data_objects);
                        break;
                }
            }
        }
        k_msleep(100);
    }
}

// public functions
bool usb_pdmon_request(uint16_t mV, uint16_t mA) {
    usb_pdmon_req_t req = { .mV = mV, .mA = mA, };
    return k_msgq_put(&usb_pdmon_msgq, &req, K_NO_WAIT) == 0;
}

bool usb_pdmon_read(uint16_t *mV, uint16_t *mA) {
    if (k_sem_take(&data.actual_lock, K_MSEC(100))) {
        LOG_ERR("unable to grab actual lock for reading");
        return false;
    } else {
        *mV = data.actual.mV;
        *mA = data.actual.mA;
        k_sem_give(&data.actual_lock);
        return true;
    }
}

#if __has_include("jabi/error.h")
#include <jabi/error.h>

int16_t jabi_metadata_custom(uint16_t idx, uint8_t *req, uint16_t req_len,
        uint8_t *resp, uint16_t *resp_len) {
    if (req_len == 0) {
        return JABI_INVALID_ARGS_ERR;
    }
    switch (req[0]) {
        case 0: {
            if (req_len != 5) {
                return JABI_INVALID_ARGS_ERR;
            }
            uint16_t mV = (req[1] << 8) | req[2]; // big-endian
            uint16_t mA = (req[3] << 8) | req[4];
            return usb_pdmon_request(mV, mA) ? 0 : JABI_PERIPHERAL_ERR;
            break;
        }

        case 1: {
            if (req_len != 1) {
                return JABI_INVALID_ARGS_ERR;
            }
            uint16_t mV, mA;
            if (usb_pdmon_read(&mV, &mA)) {
                resp[0] = (mV >> 8) & 0xFF;
                resp[1] = (mV >> 0) & 0xFF;
                resp[2] = (mA >> 8) & 0xFF;
                resp[3] = (mA >> 0) & 0xFF;
                *resp_len = 4;
                return 0;
            }
            return JABI_INVALID_ARGS_ERR;
            break;
        }

        default:
            return JABI_INVALID_ARGS_ERR;
            break;
    }
}

#endif // __has_include("jabi/error.h")

#endif // CONFIG_USBC_STACK
