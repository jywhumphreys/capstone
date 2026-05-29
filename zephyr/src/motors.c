#include "motors.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stdlib.h>

#define MOTORS_NODE DT_NODELABEL(motors)

/* PWM channels, indexed by motor_id_t (period comes from the overlay). */
static const struct pwm_dt_spec motor_pwm[MOTOR_COUNT] = {
    PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 0),
    PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 1),
    PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 2),
    PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 3),
};

/* DIR GPIOs, indexed by motor_id_t. */
static const struct gpio_dt_spec motor_dir[MOTOR_COUNT] = {
    GPIO_DT_SPEC_GET_BY_IDX(MOTORS_NODE, dir_gpios, 0),
    GPIO_DT_SPEC_GET_BY_IDX(MOTORS_NODE, dir_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(MOTORS_NODE, dir_gpios, 2),
    GPIO_DT_SPEC_GET_BY_IDX(MOTORS_NODE, dir_gpios, 3),
};

/* The right-hand wheels are mounted mirror-image to the left, so a positive
 * ("forward") command must drive their DIR pin to the opposite level. Flip
 * these entries to match your wiring/gearing if a wheel spins backwards. */
static const bool motor_invert[MOTOR_COUNT] = {
    false,  /* FL */
    true,   /* FR */
    false,  /* RL */
    true,   /* RR */
};

int motors_init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!pwm_is_ready_dt(&motor_pwm[i])) {
            return -ENODEV;
        }
        if (!gpio_is_ready_dt(&motor_dir[i])) {
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&motor_dir[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            return ret;
        }

        /* Start stopped: 0% duty. */
        ret = pwm_set_pulse_dt(&motor_pwm[i], 0);
        if (ret < 0) {
            return ret;
        }
    }
    return 0;
}

void motor_set(motor_id_t motor, int speed)
{
    if (motor >= MOTOR_COUNT) {
        return;
    }

    /* Clamp to the valid signed range. */
    if (speed > 100) {
        speed = 100;
    } else if (speed < -100) {
        speed = -100;
    }

    /* Sign -> direction, magnitude -> duty cycle (sign-magnitude mode). */
    bool forward = (speed >= 0);
    if (motor_invert[motor]) {
        forward = !forward;
    }
    gpio_pin_set_dt(&motor_dir[motor], forward ? 1 : 0);

    uint32_t mag   = (uint32_t)abs(speed);                       /* 0..100  */
    uint32_t pulse = (motor_pwm[motor].period * mag) / 100u;     /* ns      */
    pwm_set_pulse_dt(&motor_pwm[motor], pulse);
}

void motor_set_all(int fl, int fr, int rl, int rr)
{
    motor_set(MOTOR_FL, fl);
    motor_set(MOTOR_FR, fr);
    motor_set(MOTOR_RL, rl);
    motor_set(MOTOR_RR, rr);
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pwm_set_pulse_dt(&motor_pwm[i], 0);
    }
}

void mecanum_drive(int vx, int vy, int omega)
{
    /* Standard mecanum inverse kinematics (rollers at +/-45 deg).
     * vy is strafe-left positive, omega is CCW positive. */
    int w[MOTOR_COUNT];
    w[MOTOR_FL] = vx - vy - omega;
    w[MOTOR_FR] = vx + vy + omega;
    w[MOTOR_RL] = vx + vy - omega;
    w[MOTOR_RR] = vx - vy + omega;

    /* If any wheel command exceeds full scale, scale them all down by the
     * same factor so the motion direction is preserved instead of clipped. */
    int maxmag = 100;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        int m = abs(w[i]);
        if (m > maxmag) {
            maxmag = m;
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set((motor_id_t)i, (w[i] * 100) / maxmag);
    }
}
