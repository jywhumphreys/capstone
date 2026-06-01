#ifndef HOPPER_H
#define HOPPER_H

/* Linear actuator (hopper) driven by one channel of a TB6612FNG.
 *
 * Hardware: Progressive Automations PA-MC1, 12 V brushed DC, full-voltage only
 * (no PWM), with internal limit switches that cut power at end of travel — so
 * there is no stall condition. PWMA and STBY are tied high; only the two
 * direction lines (AIN1/AIN2) are driven.
 *
 * Since there is no position feedback, a move runs for a fixed timeout
 * (HOPPER_TRAVEL_MS) that is generous enough to reach the end stop, then the
 * pins are de-energized. extend()/retract() are non-blocking and arm that
 * timeout; hopper_tick() must be called periodically to enforce it. */

/* Configure the direction GPIOs. Returns 0 on success, negative errno if a pin
 * is not ready. Leaves the actuator stopped. */
int hopper_init(void);

void hopper_extend(void);   /* AIN1 = HIGH, AIN2 = LOW;  auto-stops after timeout */
void hopper_retract(void);  /* AIN1 = LOW,  AIN2 = HIGH; auto-stops after timeout */
void hopper_stop(void);     /* AIN1 = LOW,  AIN2 = LOW                            */

/* Call periodically (e.g. from the main loop) to enforce the move timeout. */
void hopper_tick(void);

#endif /* HOPPER_H */
