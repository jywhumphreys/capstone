#include "robot_comms.h"
#include "motor_control.h"
#include "usart.h"   /* huart3 */
#include "main.h"
#include <string.h>

/* ── UART RX state machine ───────────────────────────────────────────────── */
typedef enum {
    RX_WAIT_H0,
    RX_WAIT_H1,
    RX_WAIT_CMD,
    RX_RECV_DATA,
    RX_WAIT_CHK,
} RxState;

static volatile uint8_t  rx_byte;
static          RxState  rx_state      = RX_WAIT_H0;
static          uint8_t  rx_cmd;
static          uint8_t  rx_buf[DRIVE_PAYLOAD_LEN];
static          uint8_t  rx_idx;

/* ── Watchdog / odom timing ──────────────────────────────────────────────── */
static uint32_t last_drive_ms = 0;
static uint32_t last_odom_ms  = 0;

/* ── Init ────────────────────────────────────────────────────────────────── */
void robot_comms_init(void)
{
    /* Arm the first receive; subsequent bytes are re-armed in the callback */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
}

/* ── Packet processing ───────────────────────────────────────────────────── */
static void process_drive_packet(const uint8_t *payload)
{
    /* Payload: 4 × int16 big-endian, order FL FR RL RR */
    int16_t speeds[4];
    for (int i = 0; i < 4; i++) {
        speeds[i] = (int16_t)((payload[i*2] << 8) | payload[i*2 + 1]);
    }
    motor_set_all(speeds[0], speeds[1], speeds[2], speeds[3]);
    last_drive_ms = HAL_GetTick();
}

/* ── HAL UART RX callback ────────────────────────────────────────────────── */
/* Add to stm32l4xx_it.c (or wherever you have it):
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART3) robot_comms_uart_rx_callback();
 *   }
 */
void robot_comms_uart_rx_callback(void)
{
    uint8_t b = rx_byte;

    switch (rx_state) {
        case RX_WAIT_H0:
            if (b == PKT_HEADER_0) rx_state = RX_WAIT_H1;
            break;

        case RX_WAIT_H1:
            rx_state = (b == PKT_HEADER_1) ? RX_WAIT_CMD : RX_WAIT_H0;
            break;

        case RX_WAIT_CMD:
            if (b == CMD_DRIVE) {
                rx_cmd   = b;
                rx_idx   = 0;
                rx_state = RX_RECV_DATA;
            } else {
                rx_state = RX_WAIT_H0;  /* unknown command */
            }
            break;

        case RX_RECV_DATA:
            rx_buf[rx_idx++] = b;
            if (rx_idx >= DRIVE_PAYLOAD_LEN)
                rx_state = RX_WAIT_CHK;
            break;

        case RX_WAIT_CHK: {
            uint8_t chk = rx_cmd;
            for (uint8_t i = 0; i < DRIVE_PAYLOAD_LEN; i++) chk ^= rx_buf[i];
            if (chk == b)
                process_drive_packet(rx_buf);
            rx_state = RX_WAIT_H0;
            break;
        }
    }

    /* Re-arm for the next byte */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
}

/* ── Odom TX ─────────────────────────────────────────────────────────────── */
static void send_odom(void)
{
    uint8_t pkt[ODOM_PKT_LEN];
    pkt[0] = PKT_HEADER_0;
    pkt[1] = PKT_HEADER_1;
    pkt[2] = CMD_ODOM;

    int32_t counts[4] = {
        encoder_get(MOTOR_FL),
        encoder_get(MOTOR_FR),
        encoder_get(MOTOR_RL),
        encoder_get(MOTOR_RR),
    };
    memcpy(&pkt[3], counts, 16);  /* little-endian on ARM — matches Python '<4i' */

    uint8_t chk = CMD_ODOM;
    for (uint8_t i = 0; i < ODOM_PAYLOAD_LEN; i++) chk ^= pkt[3 + i];
    pkt[ODOM_PKT_LEN - 1] = chk;

    HAL_UART_Transmit(&huart3, pkt, ODOM_PKT_LEN, 10);
}

/* ── Main-loop tick (call every 1 ms or faster) ──────────────────────────── */
void robot_comms_tick(void)
{
    uint32_t now = HAL_GetTick();

    /* Drive watchdog */
    if ((now - last_drive_ms) >= DRIVE_WATCHDOG_MS) {
        motor_stop_all();
    }

    /* Send odom at 50 Hz */
    if ((now - last_odom_ms) >= 20u) {
        last_odom_ms = now;
        send_odom();
    }
}
