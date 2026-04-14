/*
 * main_additions.c — NOT a standalone compilation unit.
 * Copy the labelled snippets into main.c in the matching USER CODE sections.
 *
 * CubeMX will regenerate main.c but will preserve code inside /* USER CODE * / blocks.
 */

/* ════════════════════════════════════════════════════════════════════════════
 * 1. Includes  →  add after the existing includes in main.c
 * ════════════════════════════════════════════════════════════════════════════ */
#include "motor_control.h"
#include "robot_comms.h"

/* ════════════════════════════════════════════════════════════════════════════
 * 2. USER CODE BEGIN 2  →  after all MX_xxx_Init() calls
 * ════════════════════════════════════════════════════════════════════════════ */
motor_control_init();
robot_comms_init();

/* ════════════════════════════════════════════════════════════════════════════
 * 3. USER CODE BEGIN WHILE  →  inside the while(1) loop
 * ════════════════════════════════════════════════════════════════════════════ */
robot_comms_tick();   /* handles watchdog + 50 Hz odom TX */
/* Add HAL_Delay(1) here if you have nothing else in the loop, or rely on the
 * tick being called frequently enough from other tasks/RTOS.               */

/* ════════════════════════════════════════════════════════════════════════════
 * 4. UART RX callback  →  add to stm32l4xx_it.c (or a new user file)
 * ════════════════════════════════════════════════════════════════════════════ */
#include "robot_comms.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
        robot_comms_uart_rx_callback();
}

/* ════════════════════════════════════════════════════════════════════════════
 * 5. CubeMX configuration checklist
 * ════════════════════════════════════════════════════════════════════════════
 *
 * Clock:
 *   System clock = 80 MHz (PLL from HSI/HSE)
 *   APB1/APB2 prescaler = 1  (both 80 MHz)
 *
 * TIM1 — PWM:
 *   Mode: PWM Generation CH1..CH4
 *   PSC = 0, ARR = 3999  → 20 kHz
 *   CH1=PA8, CH2=PA9, CH3=PA10, CH4=PA11
 *   PWM Mode 1, Output Compare Preload: Enable
 *   No NVIC needed for PWM
 *
 * TIM2 — Encoder FL (32-bit):
 *   Combined Channels: Encoder Mode
 *   CH1=PA0 (AF1), CH2=PA1 (AF1)
 *   PSC=0, ARR=0xFFFFFFFF
 *   Encoder Mode: TI1FP1+TI2FP2 (x4)
 *   No interrupt needed (32-bit, no overflow)
 *
 * TIM3 — Encoder FR (16-bit):
 *   Same as TIM2 but ARR=0xFFFF
 *   CH1=PA6 (AF2), CH2=PA7 (AF2)
 *   NVIC: TIM3 global interrupt → Enabled, preempt priority 1
 *
 * TIM4 — Encoder RL (16-bit):
 *   CH1=PB6 (AF2), CH2=PB7 (AF2)
 *   NVIC: TIM4 global interrupt → Enabled, preempt priority 1
 *
 * TIM8 — Encoder RR (16-bit):
 *   CH1=PC6 (AF3), CH2=PC7 (AF3)
 *   NVIC: TIM8 update interrupt  → Enabled, preempt priority 1
 *         (TIM8_UP is a separate NVIC line on L476)
 *
 * USART3 — Jetson bridge:
 *   Mode: Asynchronous
 *   Baud: 115200, 8N1, no HW flow control
 *   TX=PC10 (AF7), RX=PC11 (AF7)
 *   NVIC: USART3 global interrupt → Enabled, preempt priority 0
 *
 * GPIO Outputs (DIR pins) — all Push-Pull, No pull, Speed=Low:
 *   PB0  label: MOTOR_FL_DIR
 *   PB1  label: MOTOR_FR_DIR
 *   PC4  label: MOTOR_RL_DIR
 *   PC5  label: MOTOR_RR_DIR
 * ════════════════════════════════════════════════════════════════════════════ */
