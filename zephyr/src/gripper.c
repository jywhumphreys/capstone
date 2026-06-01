#include "gripper.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/pwm.h>
#include <zephyr/kernel.h>

#define GRIPPER_NODE DT_NODELABEL(gripper)

static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(GRIPPER_NODE);

// 500..2500us pulse spans the full 0..270 deg travel. position 0..100 maps
// linearly across it
#define SERVO_MIN_PULSE_NS  PWM_USEC(500)
#define SERVO_MAX_PULSE_NS  PWM_USEC(2500)

// measured mechanical limits on the 0..100 scale (lower = more open). gripper
// binds past these, so positions are hard-clamped to the band
#define GRIPPER_OPEN    10
#define GRIPPER_CLOSED  30

int gripper_init(void)
{
    if (!pwm_is_ready_dt(&servo)) {
        return -ENODEV;
    }
    // don't command a position here — caller drives it, no startup jump
    return 0;
}

void gripper_set(int position)
{
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
