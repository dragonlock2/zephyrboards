#ifndef ZEPHYRBOARDS_INCLUDE_DRIVERS_LIN_H_
#define ZEPHYRBOARDS_INCLUDE_DRIVERS_LIN_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */
#define LIN_ID_MASK  0x3FU
#define LIN_NUM_ID   64U
#define LIN_MAX_DLEN 8U
/** @endcond */

/**
 * @brief LIN controller mode
 */
enum lin_mode {
    /** Commander mode */
    LIN_MODE_COMMANDER,
    /** Responder mode */
    LIN_MODE_RESPONDER,
};

/**
 * @brief LIN checksum type
 */
enum lin_checksum_type {
    /** Classic */
    LIN_CHECKSUM_CLASSIC,
    /** Enhanced */
    LIN_CHECKSUM_ENHANCED,
    /** Auto (filter only, match either) */
    LIN_CHECKSUM_AUTO,
};

/**
 * @brief LIN frame structure
 */
struct zlin_frame {
    /** LIN identifier (0-63) */
    uint8_t id;
    /** Checksum type. Use @a lin_checksum enum for assignment */
    uint8_t checksum_type;
    /** Data length */
    uint8_t data_len;
    /** @cond INTERNAL_HIDDEN */
    uint8_t res;  /* reserved/padding. */
    /** @endcond */
    /** Frame payload data */
    union {
        uint8_t data[LIN_MAX_DLEN];
        uint32_t data_32[ceiling_fraction(LIN_MAX_DLEN, sizeof(uint32_t))];
    };
};

/**
 * @brief LIN filter structure
 */
struct zlin_filter {
    /** LIN identifier to match (0-63) */
    uint8_t id;
    /** Checksum type. Use @a lin_checksum enum for assignment */
    uint8_t checksum_type;
    /** Expected data length. Use 0 to automatically determine based on valid checksum */
    uint8_t data_len;
    /** @cond INTERNAL_HIDDEN */
    uint8_t res;  /* reserved/padding. */
    /** @endcond */
};

/**
 * @brief Statically define and initialize a LIN RX message queue.
 *
 * The message queue's ring buffer contains space for @a max_frames LIN frames.
 *
 * @see lin_add_rx_filter_msgq()
 *
 * @param name       Name of the message queue.
 * @param max_frames Maximum number of LIN frames that can be queued.
 */
#define LIN_MSGQ_DEFINE(name, max_frames) \
    K_MSGQ_DEFINE(name, sizeof(struct zlin_frame), max_frames, 4)

/**
 * @brief Defines the application callback handler function signature
 *
 * Note this function may be called in an ISR
 * 
 * @param dev       Pointer to the device structure for the driver instance.
 * @param error     Status of the performed send operation. See the list of
 *                  return values for @a lin_send() and @a lin_receive() for value descriptions.
 * @param user_data User data provided when the frame was sent or received (if commander mode).
 */
typedef void (*lin_tx_callback_t)(const struct device *dev, int error, void *user_data);

/**
 * @brief Defines the application callback handler function signature for receiving
 * 
 * Note this function may be called in an ISR
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param frame     Received frame.
 * @param user_data User data provided when the filter was added.
 */
typedef void (*lin_rx_callback_t)(const struct device *dev, struct zlin_frame *frame, void *user_data);

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal driver use only, skip these in public documentation.
 */

/**
 * @brief Callback API upon setting LIN controller mode
 * See @a lin_set_mode() for argument description
 */
typedef int (*lin_set_mode_t)(const struct device *dev, enum lin_mode mode);

/**
 * @brief Callback API upon setting LIN controller bitrate
 * See @a lin_set_bitrate() for argument description
 */
typedef int (*lin_set_bitrate_t)(const struct device *dev, uint32_t bitrate);

/**
 * @brief Callback API upon sending a LIN frame
 * See @a lin_send() for argument description
 */
typedef int (*lin_send_t)(const struct device *dev, const struct zlin_frame *frame, k_timeout_t timeout,
                          lin_tx_callback_t callback, void *user_data);

/**
 * @brief Callback API upon receiving a LIN frame
 * See @a lin_receive() for argument description
 */
typedef int (*lin_receive_t)(const struct device *dev, uint8_t id, k_timeout_t timeout,
                             lin_tx_callback_t callback, void *user_data);

/**
 * @brief Callback API upon adding an RX filter
 * See @a lin_add_rx_filter() for argument description
 */
typedef int (*lin_add_rx_filter_t)(const struct device *dev, lin_rx_callback_t callback,
                                   void *user_data, const struct zlin_filter *filter);

/**
 * @brief Callback API upon removing an RX filter
 * See @a lin_remove_rx_filter() for argument description
 */
typedef void (*lin_remove_rx_filter_t)(const struct device *dev, int filter_id);

__subsystem struct lin_driver_api {
    lin_set_mode_t set_mode;
    lin_set_bitrate_t set_bitrate;
    lin_send_t send;
    lin_receive_t receive;
    lin_add_rx_filter_t add_rx_filter;
    lin_remove_rx_filter_t remove_rx_filter;
};

/** @endcond */

/**
 * @brief Set the LIN controller to the given operation mode
 * 
 * Sets the LIN controller to the given operation mode. If transitioning from responder to
 * commander mode, all pending packets to be sent are flushed.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 * @param mode Operation mode.
 *
 * @retval 0 if successful.
 * @retval -EIO General input/output error, failed to configure device.
 */
__syscall int lin_set_mode(const struct device *dev, enum lin_mode mode);

static inline int z_impl_lin_set_mode(const struct device *dev, enum lin_mode mode) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_mode(dev, mode);
}

/**
 * @brief Set the LIN controller bitrate
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param bitrate Bitrate in bits/sec
 *
 * @retval 0 if successful.
 * @retval -ENOTSUP bitrate not supported by LIN controller/transceiver combination
 * @retval -EINVAL bitrate cannot be met.
 * @retval -EIO General input/output error, failed to set bitrate.
 */
__syscall int lin_set_bitrate(const struct device *dev, uint32_t bitrate);

static inline int z_impl_lin_set_bitrate(const struct device *dev, uint32_t bitrate) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->set_bitrate(dev, bitrate);
}

/**
 * @brief Queue a LIN frame for transmission on the LIN bus
 *
 * Queue a LIN frame for transmission on the LIN bus with optional timeout and
 * completion callback function. If a receive filter with the same ID is active, that filter
 * is ignored until the frame is sent. Note that a callback is required to monitor for
 * frame errors (e.g. -EIO).
 * 
 * In commander mode, queued LIN frames are transmitted in FIFO order.
 * 
 * In responder mode, queued LIN frames are transmitted in the order requested
 * by the commander. When queueing multiple LIN frames with the same PID, they
 * are transmitted in FIFO order.
 * 
 * @param dev       Pointer to the device structure for the driver instance.
 * @param frame     LIN frame to transmit.
 * @param timeout   Timeout waiting for a empty TX queue or ``K_FOREVER``.
 * @param callback  Optional callback for when the frame was sent or a
 *                  transmission error occurred. If ``NULL``, this function is
 *                  blocking until frame is sent.
 * @param user_data User data to pass to callback function.
 *
 * @retval 0 if successful.
 * @retval -EINVAL if an invalid parameter was passed to the function.
 * @retval -ECANCELED if canceled due to a configuration change.
 * @retval -EIO if a general transmit error occurred (e.g. sent/received byte mismatch,
 *              invalid checksum).
 * @retval -EAGAIN on timeout.
 */
__syscall int lin_send(const struct device *dev, const struct zlin_frame *frame,
                       k_timeout_t timeout, lin_tx_callback_t callback, void *user_data);

static inline int z_impl_lin_send(const struct device *dev, const struct zlin_frame *frame,
                                  k_timeout_t timeout, lin_tx_callback_t callback, void *user_data) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->send(dev, frame, timeout, callback, user_data);
}

/**
 * @brief Initiate the reception of a LIN frame on the LIN bus. Only valid in commander mode
 *
 * Initiate the reception of a LIN frame on the LIN bus with optional timeout. Only valid
 * in commander mode. Use @a lin_add_rx_filter() or @a lin_add_rx_filter_msgq() to retrieve
 * the received frame. Note that a callback is required to monitor for frame errors (e.g. -EIO).
 * 
 * @param dev       Pointer to the device structure for the driver instance.
 * @param id        The ID to initiate the receive from.
 * @param timeout   Timeout waiting for a reception completion or ``K_FOREVER``.
 * @param callback  Optional callback for when the frame was received or a
 *                  transmission error occurred. If ``NULL``, this function is
 *                  blocking until frame is received.
 * @param user_data User data to pass to callback function.
 *
 * @retval 0 if successful.
 * @retval -ENOTSUP if not supported or in incorrect mode.
 * @retval -EINVAL if invalid argument or if no filter added for given id.
 * @retval -EIO if a general transmit error occurred (e.g. sent/received byte mismatch,
 *              invalid checksum, no response).
 * @retval -EAGAIN on timeout.
 */
__syscall int lin_receive(const struct device *dev, uint8_t id, k_timeout_t timeout,
                          lin_tx_callback_t callback, void *user_data);

static inline int z_impl_lin_receive(const struct device *dev, uint8_t id, k_timeout_t timeout,
                                     lin_tx_callback_t callback, void *user_data) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->receive(dev, id, timeout, callback, user_data);
}

/**
 * @brief Add a callback function for a given LIN filter
 *
 * Add a callback to LIN frames specified by a filter. When a received LIN
 * frame matching the filter is received by the LIN controller, the callback
 * function is called in interrupt context.
 *
 * If a frame matches more than one attached filter, the priority of the match
 * is hardware dependent.
 *
 * The same callback function can be used for multiple filters.
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param callback  This function is called by the LIN controller driver whenever
 *                  a frame matching the filter is received.
 * @param user_data User data to pass to callback function.
 * @param filter    Pointer to a @a zlin_filter structure defining the filter.
 *
 * @retval filter_id on success.
 * @retval -ENOSPC if there are no free filters for the given parameters (e.g. PID).
 */
static inline int lin_add_rx_filter(const struct device *dev, lin_rx_callback_t callback,
                                    void *user_data, const struct zlin_filter *filter) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->add_rx_filter(dev, callback, user_data, filter);
}

/**
 * @brief Wrapper function for adding a message queue for a given filter
 *
 * Wrapper function for @a lin_add_rx_filter() which puts received LIN frames
 * matching the filter in a message queue instead of calling a callback.
 *
 * If a frame matches more than one attached filter, the priority of the match
 * is hardware dependent.
 *
 * The same message queue can be used for multiple filters.
 *
 * @note The message queue must be initialized before calling this function and
 * the caller must have appropriate permissions on it.
 *
 * @param dev    Pointer to the device structure for the driver instance.
 * @param msgq   Pointer to the already initialized @a k_msgq struct.
 * @param filter Pointer to a @a zlin_filter structure defining the filter.
 *
 * @retval filter_id on success.
 * @retval -ENOSPC if there are no free filters for the given parameters (e.g. PID).
 */
__syscall int lin_add_rx_filter_msgq(const struct device *dev, struct k_msgq *msgq,
                                     const struct zlin_filter *filter);

/**
 * @brief Remove a LIN RX filter
 *
 * This routine removes a LIN RX filter based on the filter ID returned by @a
 * lin_add_rx_filter() or @a lin_add_rx_filter_msgq().
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param filter_id Filter ID
 */
__syscall void lin_remove_rx_filter(const struct device *dev, int filter_id);

static inline void z_impl_lin_remove_rx_filter(const struct device *dev, int filter_id) {
    const struct lin_driver_api *api = (const struct lin_driver_api *)dev->api;
    return api->remove_rx_filter(dev, filter_id);
}

#ifdef __cplusplus
}
#endif

#include <syscalls/lin.h>

#endif // ZEPHYRBOARDS_INCLUDE_DRIVERS_LIN_H_
