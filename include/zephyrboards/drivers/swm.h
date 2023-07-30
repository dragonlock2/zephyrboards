#ifndef ZEPHYRBOARDS_INCLUDE_DRIVERS_SWM_H_
#define ZEPHYRBOARDS_INCLUDE_DRIVERS_SWM_H_

#include <zephyr/device.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/* used to identify the pin to assign a function to */
typedef void *swm_pin_t;

/* function to be assigned */
typedef void *swm_function_t;

typedef int (*swm_assign_fn)(const struct device *dev,
                             swm_pin_t pin,
                             swm_function_t func);

struct swm_driver_api {
    swm_assign_fn assign;
};

static inline int swm_assign(const struct device *dev,
                             swm_pin_t pin,
                             swm_function_t func) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    const struct swm_driver_api *api =
        (const struct swm_driver_api*) dev->api;

    return api->assign(dev, pin, func);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYRBOARDS_INCLUDE_DRIVERS_SWM_H_ */