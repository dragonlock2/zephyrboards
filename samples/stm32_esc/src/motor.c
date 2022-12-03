#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <stm32f1xx.h>
#include <stm32f1xx_ll_tim.h>
#include "motor.h"

/* private constants */
#define MOTOR_OPEN_LOOP_CYCLES (10) // increase if trouble starting or sudden direction changes
#define MOTOR_MAX_BEMF_TIME    (K_MSEC(100)) // compromise max torque and startup capability
#define MOTOR_REQ_BUFFER_LEN   (3)
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
    MOTOR_STATE_COUNT,
} motor_state_E;

typedef struct {
    motor_drive_E drive[MOTOR_TERM_COUNT];
    motor_term_E bemf_gpio;
    bool bemf_level;
} motor_lut_S;

static const motor_lut_S MOTOR_DRIVE_LUT[MOTOR_STATE_COUNT] = {
    [MOTOR_STATE_0]   = {.drive = {MOTOR_DRIVE_POS, MOTOR_DRIVE_NEG, MOTOR_DRIVE_HIZ}, .bemf_gpio = MOTOR_TERM_C, .bemf_level = false},
    [MOTOR_STATE_60]  = {.drive = {MOTOR_DRIVE_POS, MOTOR_DRIVE_HIZ, MOTOR_DRIVE_NEG}, .bemf_gpio = MOTOR_TERM_B, .bemf_level = true },
    [MOTOR_STATE_120] = {.drive = {MOTOR_DRIVE_HIZ, MOTOR_DRIVE_POS, MOTOR_DRIVE_NEG}, .bemf_gpio = MOTOR_TERM_A, .bemf_level = false},
    [MOTOR_STATE_180] = {.drive = {MOTOR_DRIVE_NEG, MOTOR_DRIVE_POS, MOTOR_DRIVE_HIZ}, .bemf_gpio = MOTOR_TERM_C, .bemf_level = true },
    [MOTOR_STATE_240] = {.drive = {MOTOR_DRIVE_NEG, MOTOR_DRIVE_HIZ, MOTOR_DRIVE_POS}, .bemf_gpio = MOTOR_TERM_B, .bemf_level = false},
    [MOTOR_STATE_300] = {.drive = {MOTOR_DRIVE_HIZ, MOTOR_DRIVE_NEG, MOTOR_DRIVE_POS}, .bemf_gpio = MOTOR_TERM_A, .bemf_level = true },
};

/* private data */
typedef struct {
    struct pwm_dt_spec hsd;
    struct pwm_dt_spec lsd;
    struct gpio_dt_spec bemf;
    struct gpio_callback bemf_cb_data;
    TIM_TypeDef *timer;
} motor_term_S;

typedef struct {
    uint16_t pwm;
    bool forward;
} motor_request_S;

typedef struct {
    motor_term_S terms[MOTOR_TERM_COUNT];
    uint32_t pwm_period;
    uint32_t pwm_pulse;
    bool forward;

    uint32_t cycle_start;
    uint32_t cycle_time;

    char req_buffer[MOTOR_REQ_BUFFER_LEN * sizeof(motor_request_S)];
    struct k_msgq reqs;

    struct k_timer timeout_timer;
    struct k_timer offset_timer;

    bool closed_loop;
    uint32_t cycles;
    motor_state_E state;
} motor_data_S;

static motor_data_S motor_data = {
    .terms = {
        [MOTOR_TERM_A] = {
            .hsd   = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_a), 0),
            .lsd   = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_a), 1),
            .bemf  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bemf_trig), gpios, 0),
            .timer = TIM2,

        },
        [MOTOR_TERM_B] = {
            .hsd   = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_b), 0),
            .lsd   = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_b), 1),
            .bemf  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bemf_trig), gpios, 1),
            .timer = TIM3,
        },
        [MOTOR_TERM_C] = {
            .hsd   = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_c), 0),
            .lsd   = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_c), 1),
            .bemf  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bemf_trig), gpios, 2),
            .timer = TIM4,
        },
    }
};

/* private helpers */
static void motor_feedback_cb(struct k_timer *timer);

static void motor_drive_write(motor_state_E state, uint16_t pwm) {
    // ATA6844 specific driving
    // duty cycle == CCRx / (ARR + 1) (min ~230 before distortion)
    for (motor_term_E i = 0; i < MOTOR_TERM_COUNT; i++) {
        switch (MOTOR_DRIVE_LUT[state].drive[i]) {
            case MOTOR_DRIVE_POS:
                motor_data.terms[i].timer->CCR1 = pwm;
                motor_data.terms[i].timer->CCR2 = pwm;
                break;

            case MOTOR_DRIVE_NEG:
                motor_data.terms[i].timer->CCR1 = 0;
                motor_data.terms[i].timer->CCR2 = 0;
                break;

            case MOTOR_DRIVE_HIZ:
                motor_data.terms[i].timer->CCR1 = motor_data.pwm_period;
                motor_data.terms[i].timer->CCR2 = 0;
                break;
        }
    }
}

static void motor_bemf_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    motor_feedback_cb(NULL);
}

static void motor_step(struct k_timer *timer) {
    // check if new request
    motor_request_S req;
    if (k_msgq_get(&motor_data.reqs, &req, K_NO_WAIT) == 0) {
        if (motor_data.forward != req.forward) {
            motor_data.cycles = 0;
            motor_data.closed_loop = false;
        }
        motor_data.pwm_pulse = req.pwm;
        motor_data.forward = req.forward;
    }

    // update state, cycle time
    if (motor_data.forward) {
        motor_data.state = motor_data.state == MOTOR_STATE_300 ? MOTOR_STATE_0 : motor_data.state + 1;
    } else {
        motor_data.state = motor_data.state == MOTOR_STATE_0 ? MOTOR_STATE_300 : motor_data.state - 1;
    }
    if (motor_data.state == MOTOR_STATE_0) {
        uint32_t t = k_cycle_get_32();
        motor_data.cycle_time  = k_cyc_to_us_floor32(t - motor_data.cycle_start);
        motor_data.cycle_start = t;
    }

    motor_drive_write(motor_data.state, motor_data.pwm_pulse);

    // setup feedback
    k_timer_start(&motor_data.timeout_timer, MOTOR_MAX_BEMF_TIME, K_NO_WAIT);
    if (motor_data.forward) {
        gpio_pin_interrupt_configure_dt(
            &motor_data.terms[MOTOR_DRIVE_LUT[motor_data.state].bemf_gpio].bemf,
            MOTOR_DRIVE_LUT[motor_data.state].bemf_level ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_EDGE_TO_INACTIVE);
    } else {
        gpio_pin_interrupt_configure_dt(
            &motor_data.terms[MOTOR_DRIVE_LUT[motor_data.state].bemf_gpio].bemf,
            MOTOR_DRIVE_LUT[motor_data.state].bemf_level ? GPIO_INT_EDGE_TO_INACTIVE : GPIO_INT_EDGE_TO_ACTIVE);
    }
}

static void motor_feedback_cb(struct k_timer *timer) {
    gpio_pin_interrupt_configure_dt(
        &motor_data.terms[MOTOR_DRIVE_LUT[motor_data.state].bemf_gpio].bemf,
        GPIO_INT_DISABLE);

    if (timer == NULL) {
        if (motor_data.state == MOTOR_STATE_0) {
            motor_data.cycles++;
            if (motor_data.cycles >= MOTOR_OPEN_LOOP_CYCLES) {
                motor_data.closed_loop = true;
            }
        }
    } else {
        motor_data.cycles = 0;
        motor_data.closed_loop = false;
    }

    if (motor_data.closed_loop) {
        k_timer_start(&motor_data.offset_timer, K_USEC(motor_data.cycle_time / 12), K_NO_WAIT);
    } else {
        motor_step(NULL);
    }
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
        // setup PWM
        pwm_set_pulse_dt(&motor_data.terms[i].hsd, PWM_USEC(1)); // 1us to ensure output enabled
        pwm_set_pulse_dt(&motor_data.terms[i].lsd, PWM_USEC(1));

        // setup interrupts
        gpio_pin_configure_dt(&motor_data.terms[i].bemf, GPIO_INPUT);
        gpio_init_callback(&motor_data.terms[i].bemf_cb_data, motor_bemf_cb, BIT(motor_data.terms[i].bemf.pin));
        gpio_add_callback(motor_data.terms[i].bemf.port, &motor_data.terms[i].bemf_cb_data);
    }
    motor_data.pwm_period = motor_data.terms[MOTOR_TERM_A].timer->ARR + 1; // assume all same
    motor_data.pwm_pulse = 0;
    motor_data.forward = true;

    // configure timers
    k_timer_init(&motor_data.timeout_timer, motor_feedback_cb, NULL);
    k_timer_init(&motor_data.offset_timer,  motor_step, NULL);

    // start control loop
    k_msgq_init(&motor_data.reqs, motor_data.req_buffer, sizeof(motor_request_S), MOTOR_REQ_BUFFER_LEN);
    motor_data.cycle_start = k_cycle_get_32();
    motor_data.closed_loop = false;
    motor_data.cycles = 0;
    motor_data.state = MOTOR_STATE_0;
    motor_step(NULL);
}

void motor_write(double duty, bool forward) {
    duty = duty < 0.0 ? 0.0 : (duty > 1.0 ? 1.0 : duty); // constrain [0,1]
    motor_request_S req = {
        .pwm = duty * motor_data.pwm_period,
        .forward = forward,
    };
    k_msgq_put(&motor_data.reqs, &req, K_FOREVER);
}

uint32_t motor_cycle_time_us() {
    return motor_data.cycle_time; // assume written in one instruction :P
}
