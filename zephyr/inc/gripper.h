#ifndef GRIPPER_H
#define GRIPPER_H

/* Configure the servo PWM. Returns 0 on success, negative errno if the PWM
 * device is not ready. Leaves the gripper closed. */
int gripper_init(void);

/* Command the gripper position on the 0..100 scale (lower = more open).
 * Hard-clamped to the calibrated travel limits, so it can never drive the
 * servo past its mechanical open/closed stops. */
void gripper_set(int position);

/* Bench test: sweep the gripper open <-> closed within the calibrated limits.
 * Call repeatedly from a loop. */
void gripper_test(void);

#endif /* GRIPPER_H */
