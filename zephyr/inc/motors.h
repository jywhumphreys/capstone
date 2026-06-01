#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

// wheel order matches the comms protocol: 0=FL, 1=FR, 2=RL, 3=RR
typedef enum {
    MOTOR_FL = 0,
    MOTOR_FR = 1,
    MOTOR_RL = 2,
    MOTOR_RR = 3,
    MOTOR_COUNT = 4,
} motor_id_t;

// configure pwm + dir pins, leave all stopped. 0 ok, <0 errno
int motors_init(void);

// sign-magnitude: speed -100..100, sign -> dir pin, magnitude -> duty. clamped
void motor_set(motor_id_t motor, int speed);

void motor_set_all(int fl, int fr, int rl, int rr);
void motor_stop_all(void);

// mecanum inverse kinematics. vx fwd, vy strafe-left, omega ccw, all -100..100.
// scaled down proportionally if any wheel would exceed full scale
void mecanum_drive(int vx, int vy, int omega);

#endif /* MOTORS_H */
