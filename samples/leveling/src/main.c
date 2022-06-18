#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/pwm.h>
#include <usb/usb_device.h>
#include <math.h>

#include "vector.h"

// devices
const struct device *leds[] = {
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(red_pwm_led))),
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(green_pwm_led))),
    DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(blue_pwm_led))),
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);
struct gpio_callback btn_cb_data;

const struct device *lsm6dsm = DEVICE_DT_GET_ANY(st_lsm6dsl); // compatible!

// RGB
#define RED_NODE   DT_ALIAS(red_pwm_led)
#define GREEN_NODE DT_ALIAS(green_pwm_led)
#define BLUE_NODE  DT_ALIAS(blue_pwm_led)

#define PWM_PERIOD PWM_MSEC(10)

void set_rgb(double r, double g, double b) {
    uint32_t rp = round(r * PWM_PERIOD);
    uint32_t gp = round(g * PWM_PERIOD);
    uint32_t bp = round(b * PWM_PERIOD);

    pwm_set(leds[0], DT_PWMS_CHANNEL(RED_NODE),
        PWM_PERIOD, rp, DT_PWMS_FLAGS(RED_NODE));
    pwm_set(leds[1], DT_PWMS_CHANNEL(GREEN_NODE),
        PWM_PERIOD, gp, DT_PWMS_FLAGS(GREEN_NODE));
    pwm_set(leds[2], DT_PWMS_CHANNEL(BLUE_NODE),
        PWM_PERIOD, bp, DT_PWMS_FLAGS(BLUE_NODE));
}

// angles
#define ANGLE_THRESHOLD 1 // degree

struct accel_vec target = { // normalized
    .x = 0,
    .y = 0,
    .z = -1,
};

bool update_target = false;

void process_button(const struct device *port, 
                    struct gpio_callback *cb, gpio_port_pins_t pins) {
    update_target = true;
}

void process_reading(const struct device *dev, 
                     const struct sensor_trigger *trig) {
    // get reading
    struct sensor_value x_out, y_out, z_out;

    sensor_sample_fetch_chan(lsm6dsm, SENSOR_CHAN_ACCEL_XYZ);
    sensor_channel_get(lsm6dsm, SENSOR_CHAN_ACCEL_X, &x_out);
    sensor_channel_get(lsm6dsm, SENSOR_CHAN_ACCEL_Y, &y_out);
    sensor_channel_get(lsm6dsm, SENSOR_CHAN_ACCEL_Z, &z_out);

    struct accel_vec meas = {
        .x = sensor_value_to_double(&x_out),
        .y = sensor_value_to_double(&y_out),
        .z = sensor_value_to_double(&z_out),
    };

    normalize(&meas);

    // update target if needed
    if (update_target) {
        target = meas;
        update_target = false;
    }

    // check angle
    double ang = angle(&meas, &target);

    if (ang < ANGLE_THRESHOLD) {
        set_rgb(0, 0.1, 0);
    } else {
        set_rgb(0.1, 0, 0);
    }

    printk("angle: %.3f°\r\n", ang);
}

int main() {
    // enable usb console
    usb_enable(NULL);

    // do a little flash
    set_rgb(1, 0, 0);
    k_msleep(50);
    set_rgb(0, 1, 0);
    k_msleep(50);
    set_rgb(0, 0, 1);
    k_msleep(50);
    set_rgb(0, 0, 0);

    // configure button
    gpio_pin_configure_dt(&btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb_data, process_button, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb_data);

    // configure LSM6DSM
    struct sensor_value odr = {.val1 = 12, .val2 = 500000}; // 12.5Hz
    sensor_attr_set(lsm6dsm,
        SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };
    sensor_trigger_set(lsm6dsm, &trig, process_reading);

    sensor_sample_fetch(lsm6dsm); // start fetching

    return 0;
}
