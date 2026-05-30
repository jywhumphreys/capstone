#include "gripper.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/pwm.h>
#include <zephyr/kernel.h>

#define GRIPPER_NODE DT_NODELABEL(gripper)

static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(GRIPPER_NODE);

/* Servo pulse range: 500 us .. 2500 us spans the full 0..270 deg travel
 * (per the servo datasheet). position 0..100 maps linearly across it. */
#define SERVO_MIN_PULSE_NS  PWM_USEC(500)
#define SERVO_MAX_PULSE_NS  PWM_USEC(2500)

/* Calibrated mechanical limits on the 0..100 scale (measured on the bench).
 * The gripper binds past these, so positions are hard-clamped to this band —
 * lower = more open. Never command outside [GRIPPER_OPEN, GRIPPER_CLOSED]. */
#define GRIPPER_OPEN    10   /* max open   */
#define GRIPPER_CLOSED  60   /* max closed */

int gripper_init(void)
{
    if (!pwm_is_ready_dt(&servo)) {
        return -ENODEV;
    }

    /* Don't command a position here — the caller drives it, so the servo
     * goes straight to the first commanded position with no startup move. */
    return 0;
}

void gripper_set(int position)
{
    /* Hard-clamp to the calibrated travel so we never drive into a stop. */
    if (position < GRIPPER_OPEN) {
        position = GRIPPER_OPEN;
    } else if (position > GRIPPER_CLOSED) {
        position = GRIPPER_CLOSED;
    }

    uint32_t span  = SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS;
    uint32_t pulse = SERVO_MIN_PULSE_NS + (span * (uint32_t)position) / 100u;
    pwm_set_pulse_dt(&servo, pulse);
}

void gripper_test(void)
{
    /* Sweep between the calibrated open and closed limits — never beyond. */
    printk("gripper: open\n");
    gripper_set(GRIPPER_OPEN);
    k_msleep(1200);

    printk("gripper: closing\n");
    for (int p = GRIPPER_OPEN; p <= GRIPPER_CLOSED; p += 2) {
        gripper_set(p);
        k_msleep(60);
    }
    printk("gripper: closed\n");
    k_msleep(1200);

    printk("gripper: opening\n");
    for (int p = GRIPPER_CLOSED; p >= GRIPPER_OPEN; p -= 2) {
        gripper_set(p);
        k_msleep(60);
    }
    k_msleep(1200);
}
