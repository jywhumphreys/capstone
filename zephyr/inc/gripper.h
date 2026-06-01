#ifndef GRIPPER_H
#define GRIPPER_H

// configure the servo pwm, leave it idle. 0 ok, <0 errno
int gripper_init(void);

// position 0..100, lower = more open. clamped to the calibrated travel band
void gripper_set(int position);

// sweep open <-> closed within the limits. call in a loop
void gripper_test(void);

#endif /* GRIPPER_H */
