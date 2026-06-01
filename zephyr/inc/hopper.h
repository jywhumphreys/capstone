#ifndef HOPPER_H
#define HOPPER_H

// linear actuator (hopper) on one channel of a TB6612FNG. PA-MC1, 12V, full
// voltage only (no pwm), internal limit switches so no stall. PWMA + STBY tied
// high, only AIN1/AIN2 driven. no feedback, so a move runs for a fixed timeout
// then the pins are released — extend/retract arm it, hopper_tick enforces it

int hopper_init(void);      // configure dir pins, leave stopped. 0 ok, <0 errno

void hopper_extend(void);   // ain1 high, ain2 low
void hopper_retract(void);  // ain1 low,  ain2 high
void hopper_stop(void);     // both low

void hopper_tick(void);     // call periodically to enforce the move timeout

#endif /* HOPPER_H */
