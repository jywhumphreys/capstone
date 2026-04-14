#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/* ── Motor indices ───────────────────────────────────────────────────────── */
#define MOTOR_FL  0u
#define MOTOR_FR  1u
#define MOTOR_RL  2u
#define MOTOR_RR  3u

/*
 * ── CubeMX peripheral assignments ────────────────────────────────────────
 *
 *  PWM (TIM1, 20 kHz — ARR = 3999, PSC = 0, 80 MHz APB2):
 *    TIM1_CH1  PA8   Motor FL
 *    TIM1_CH2  PA9   Motor FR
 *    TIM1_CH3  PA10  Motor RL
 *    TIM1_CH4  PA11  Motor RR  (PA11 = USB-DM; safe if USB Device unused)
 *
 *  Direction GPIOs (Output, Push-Pull, no pull):
 *    MOTOR_FL_DIR  PB0   (label in CubeMX: MOTOR_FL_DIR)
 *    MOTOR_FR_DIR  PB1
 *    MOTOR_RL_DIR  PC4
 *    MOTOR_RR_DIR  PC5
 *
 *  Encoders (Encoder Mode, TI1FP1+TI2FP2, both edges):
 *    TIM2  PA0(CH1) PA1(CH2)   Motor FL  [32-bit — no overflow handling needed]
 *    TIM3  PA6(CH1) PA7(CH2)   Motor FR  [16-bit — overflow tracked in ISR]
 *    TIM4  PB6(CH1) PB7(CH2)   Motor RL  [16-bit]
 *    TIM8  PC6(CH1) PC7(CH2)   Motor RR  [16-bit]
 *
 *  NVIC to enable in CubeMX:
 *    TIM3 global interrupt
 *    TIM4 global interrupt
 *    TIM8 update interrupt   (separate line from TIM8_CC on L476)
 */

void    motor_control_init(void);

/* speed: –1000 (full reverse) … +1000 (full forward) */
void    motor_set(uint8_t motor, int16_t speed);
void    motor_set_all(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void    motor_stop_all(void);

/* Cumulative encoder ticks since boot (or last reset) */
int32_t encoder_get(uint8_t motor);
void    encoder_reset_all(void);

#endif /* MOTOR_CONTROL_H */
