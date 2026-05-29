#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

/* Wheel index — matches the comms protocol order (0=FL, 1=FR, 2=RL, 3=RR). */
typedef enum {
    MOTOR_FL = 0,
    MOTOR_FR = 1,
    MOTOR_RL = 2,
    MOTOR_RR = 3,
    MOTOR_COUNT = 4,
} motor_id_t;

/* Configure all PWM channels and DIR GPIOs. Returns 0 on success, negative
 * errno if a device is not ready. Leaves every motor stopped. */
int motors_init(void);

/* Drive one motor in sign-magnitude mode.
 *   speed: signed, -100 (full reverse) .. +100 (full forward).
 *          Sign sets the DIR pin, magnitude sets the PWM duty cycle.
 *          Out-of-range values are clamped. */
void motor_set(motor_id_t motor, int speed);

/* Convenience: set all four wheels at once. */
void motor_set_all(int fl, int fr, int rl, int rr);

/* Cut PWM on every motor (duty 0%). */
void motor_stop_all(void);

/* Mecanum inverse kinematics.
 *   vx    : forward (+) / backward (-)
 *   vy    : strafe left (+) / right (-)
 *   omega : rotate CCW (+) / CW (-)
 * All inputs in normalized command units (-100..100). The resulting wheel
 * commands are scaled down proportionally if any exceeds full scale, so the
 * commanded motion direction is preserved rather than clipped. */
void mecanum_drive(int vx, int vy, int omega);

#endif /* MOTORS_H */
