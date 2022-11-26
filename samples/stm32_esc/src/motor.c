#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "motor.h"

/* private constants */
typedef enum {
    MOTOR_TERM_A = 0,
    MOTOR_TERM_B,
    MOTOR_TERM_C,
    MOTOR_TERM_COUNT,
} motor_term_E;

typedef enum {
    MOTOR_DRIVE_POS,
    MOTOR_DRIVE_NEG,
    MOTOR_DRIVE_HIZ,
} motor_drive_E;

typedef enum {
    MOTOR_STATE_0 = 0,
    MOTOR_STATE_60,
    MOTOR_STATE_120,
    MOTOR_STATE_180,
    MOTOR_STATE_240,
    MOTOR_STATE_300,
    MOTOR_STATE_COAST,
    MOTOR_STATE_COUNT,
} motor_state_E;

static const motor_drive_E MOTOR_DRIVE_LUT[MOTOR_STATE_COUNT][MOTOR_TERM_COUNT] = {
    {MOTOR_DRIVE_POS, MOTOR_DRIVE_NEG, MOTOR_DRIVE_HIZ},
    {MOTOR_DRIVE_POS, MOTOR_DRIVE_HIZ, MOTOR_DRIVE_NEG},
    {MOTOR_DRIVE_HIZ, MOTOR_DRIVE_POS, MOTOR_DRIVE_NEG},
    {MOTOR_DRIVE_NEG, MOTOR_DRIVE_POS, MOTOR_DRIVE_HIZ},
    {MOTOR_DRIVE_NEG, MOTOR_DRIVE_HIZ, MOTOR_DRIVE_POS},
    {MOTOR_DRIVE_HIZ, MOTOR_DRIVE_NEG, MOTOR_DRIVE_POS},
    {MOTOR_DRIVE_HIZ, MOTOR_DRIVE_HIZ, MOTOR_DRIVE_HIZ},
};

/* private data */
typedef struct {
    struct pwm_dt_spec  hsd;
    struct pwm_dt_spec  lsd;
    struct gpio_dt_spec bemf;
} motor_term_S;

typedef struct {
    motor_term_S terms[MOTOR_TERM_COUNT];

    motor_state_E state;
} motor_data_S;

static motor_data_S motor_data = {
    .terms = {
        [MOTOR_TERM_A] = {
            .hsd  = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_a), 0),
            .lsd  = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_a), 1),
            .bemf = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bemf_trig), gpios, 0),
        },
        [MOTOR_TERM_B] = {
            .hsd  = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_b), 0),
            .lsd  = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_b), 1),
            .bemf = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bemf_trig), gpios, 1),
        },
        [MOTOR_TERM_C] = {
            .hsd  = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_c), 0),
            .lsd  = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_c), 1),
            .bemf = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bemf_trig), gpios, 2),
        },
    }
};

/* private helpers */
static void motor_drive_write(motor_state_E state, uint16_t pwm) {
    // TODO make sure HiZ is actually HiZ, check freq (it might be half tbh) and stuff

    // TODO see how fast func is with zephyr API
}

/* public functions */
void motor_init() {
    // ATA6844 control pins
    const struct gpio_dt_spec coast = GPIO_DT_SPEC_GET(DT_NODELABEL(coast), gpios);
    const struct gpio_dt_spec sleep = GPIO_DT_SPEC_GET(DT_NODELABEL(sleep), gpios);
    gpio_pin_configure_dt(&coast, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&sleep, GPIO_OUTPUT_INACTIVE);

    // configure pins
    for (motor_term_E i = 0; i < MOTOR_TERM_COUNT; i++) {
        // TODO setup PWM, get raw counter reg and use as the max
        pwm_set_pulse_dt(&motor_data.terms[i].hsd, PWM_USEC(10));
        pwm_set_pulse_dt(&motor_data.terms[i].lsd, PWM_USEC(10));

        // TODO setup interrupt
        gpio_pin_configure_dt(&motor_data.terms[i].bemf, GPIO_INPUT);
    }

    motor_drive_write(MOTOR_STATE_COAST, 0);

    // TODO IPD, just align to one pole
    // TODO startup procedure after IPD? needed?
    // TODO braking? flipping dir while moving?
    // TODO add time for last revolution API
}

void motor_write(double duty, bool forward) {
}
