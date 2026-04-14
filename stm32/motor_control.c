#include "motor_control.h"
#include "main.h"   /* GPIO pin labels from CubeMX */
#include "tim.h"    /* htim1, htim2, htim3, htim4, htim8 */

/* ── PWM ─────────────────────────────────────────────────────────────────── */
/* Must match ARR configured in CubeMX for TIM1.
 * At 80 MHz with PSC=0: ARR = 80_000_000 / 20_000 – 1 = 3999            */
#define PWM_PERIOD  3999u

/* ── Encoder overflow tracking for 16-bit timers ────────────────────────── */
/* TIM2 is 32-bit → index 0 unused (direct read).
 * TIM3→idx 1, TIM4→idx 2, TIM8→idx 3                                     */
static volatile int32_t enc_overflow[4] = {0, 0, 0, 0};

/* ── Init ────────────────────────────────────────────────────────────────── */
void motor_control_init(void)
{
    /* PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Encoders */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    /* Enable update interrupts on 16-bit encoder timers for overflow tracking.
     * Requires TIM3/TIM4/TIM8 global IRQs enabled in NVIC (CubeMX).         */
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

    motor_stop_all();
}

/* ── Motor control ───────────────────────────────────────────────────────── */
void motor_set(uint8_t motor, int16_t speed)
{
    /* Clamp */
    if (speed >  1000) speed =  1000;
    if (speed < -1000) speed = -1000;

    GPIO_TypeDef   *dir_port;
    uint16_t        dir_pin;
    TIM_HandleTypeDef *htim;
    uint32_t        channel;

    switch (motor) {
        case MOTOR_FL:
            dir_port = MOTOR_FL_DIR_GPIO_Port; dir_pin = MOTOR_FL_DIR_Pin;
            htim = &htim1; channel = TIM_CHANNEL_1; break;
        case MOTOR_FR:
            dir_port = MOTOR_FR_DIR_GPIO_Port; dir_pin = MOTOR_FR_DIR_Pin;
            htim = &htim1; channel = TIM_CHANNEL_2; break;
        case MOTOR_RL:
            dir_port = MOTOR_RL_DIR_GPIO_Port; dir_pin = MOTOR_RL_DIR_Pin;
            htim = &htim1; channel = TIM_CHANNEL_3; break;
        case MOTOR_RR:
            dir_port = MOTOR_RR_DIR_GPIO_Port; dir_pin = MOTOR_RR_DIR_Pin;
            htim = &htim1; channel = TIM_CHANNEL_4; break;
        default: return;
    }

    uint32_t duty = (uint32_t)((speed < 0 ? -speed : speed) * PWM_PERIOD) / 1000u;
    HAL_GPIO_WritePin(dir_port, dir_pin,
                      speed >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, channel, duty);
}

void motor_set_all(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    motor_set(MOTOR_FL, fl);
    motor_set(MOTOR_FR, fr);
    motor_set(MOTOR_RL, rl);
    motor_set(MOTOR_RR, rr);
}

void motor_stop_all(void)
{
    motor_set_all(0, 0, 0, 0);
}

/* ── Encoder reads ───────────────────────────────────────────────────────── */
int32_t encoder_get(uint8_t motor)
{
    switch (motor) {
        case MOTOR_FL:
            /* TIM2 is 32-bit: counter is already a full signed value */
            return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        case MOTOR_FR:
            return (int32_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim3))
                   + enc_overflow[1] * 65536;
        case MOTOR_RL:
            return (int32_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim4))
                   + enc_overflow[2] * 65536;
        case MOTOR_RR:
            return (int32_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim8))
                   + enc_overflow[3] * 65536;
        default: return 0;
    }
}

void encoder_reset_all(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    for (int i = 0; i < 4; i++) enc_overflow[i] = 0;
}

/* ── Overflow ISR callback ───────────────────────────────────────────────── */
/* CubeMX generates TIM3/TIM4/TIM8_UP IRQ handlers that call
 * HAL_TIM_IRQHandler, which in turn calls this weak callback.
 * If you already have this callback elsewhere, merge the bodies.          */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t idx;
    if      (htim->Instance == TIM3) idx = 1;
    else if (htim->Instance == TIM4) idx = 2;
    else if (htim->Instance == TIM8) idx = 3;
    else return;

    /* CR1.DIR = 1 → counting down (underflow); 0 → counting up (overflow) */
    if (htim->Instance->CR1 & TIM_CR1_DIR)
        enc_overflow[idx]--;
    else
        enc_overflow[idx]++;
}
