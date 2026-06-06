#include "comms.h"
#include "comms_protocol.h"
#include "motors.h"
#include "gripper.h"
#include "hopper.h"
#include "servo.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <string.h>

typedef enum {
    RX_WAIT_H0,
    RX_WAIT_H1,
    RX_WAIT_CMD,
    RX_WAIT_LEN,
    RX_RECV_DATA,
    RX_WAIT_CHK,
} RxState;

static const struct device *uart_dev;

static RxState rx_state = RX_WAIT_H0;
static uint8_t rx_cmd;
static uint8_t rx_len;
static uint8_t rx_buf[RX_MAX_PAYLOAD];
static uint8_t rx_idx;

// the isr only parses framing. a finished packet is stashed here and applied
// in uart_tick (thread context) — the arm servo bus does blocking uart i/o
// that must not run in an isr
static volatile bool pkt_ready;
static uint8_t pkt_cmd;
static uint8_t pkt_len;
static uint8_t pkt_buf[RX_MAX_PAYLOAD];

static uint32_t last_drive_ms;
static uint32_t last_odom_ms;

static void uart_rx_callback(const struct device *dev, void *user_data);
static void uart_output(void);

static bool cmd_known(uint8_t cmd)
{
    return cmd == CMD_DRIVE || cmd == CMD_GRIPPER ||
           cmd == CMD_HOPPER || cmd == CMD_ARM;
}

void uart_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
    if (!device_is_ready(uart_dev)) {
        return;
    }
    uart_irq_callback_user_data_set(uart_dev, uart_rx_callback, NULL);
    uart_irq_rx_enable(uart_dev);
}

// isr: parse framing only, hand a finished packet to uart_tick
static void uart_rx_callback(const struct device *dev, void *user_data)
{
    if (!uart_irq_update(dev)) return;
    if (!uart_irq_rx_ready(dev)) return;

    uint8_t b;
    uart_fifo_read(dev, &b, 1);

    switch (rx_state) {
    case RX_WAIT_H0:
        if (b == PKT_HEADER_0) rx_state = RX_WAIT_H1;
        break;

    case RX_WAIT_H1:
        rx_state = (b == PKT_HEADER_1) ? RX_WAIT_CMD : RX_WAIT_H0;
        break;

    case RX_WAIT_CMD:
        if (cmd_known(b)) {
            rx_cmd = b;
            rx_state = RX_WAIT_LEN;
        } else {
            rx_state = RX_WAIT_H0;
        }
        break;

    case RX_WAIT_LEN:
        if (b > RX_MAX_PAYLOAD) {
            rx_state = RX_WAIT_H0;
        } else {
            rx_len = b;
            rx_idx = 0;
            rx_state = (b == 0) ? RX_WAIT_CHK : RX_RECV_DATA;
        }
        break;

    case RX_RECV_DATA:
        rx_buf[rx_idx++] = b;
        if (rx_idx >= rx_len) rx_state = RX_WAIT_CHK;
        break;

    case RX_WAIT_CHK: {
        uint8_t chk = rx_cmd ^ rx_len;
        for (uint8_t i = 0; i < rx_len; i++) chk ^= rx_buf[i];
        // drop the new packet if the last one isn't consumed yet
        if (chk == b && !pkt_ready) {
            pkt_cmd = rx_cmd;
            pkt_len = rx_len;
            memcpy(pkt_buf, rx_buf, rx_len);
            pkt_ready = true;
        }
        rx_state = RX_WAIT_H0;
        break;
    }
    }
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t leu16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// thread context (called from uart_tick) — blocking calls are fine here
static void apply_packet(uint8_t cmd, const uint8_t *p, uint8_t len)
{
    switch (cmd) {
    case CMD_DRIVE:
        if (len != DRIVE_PAYLOAD_LEN) break;
        motor_set_all(le16(p) / 10, le16(p + 2) / 10,      // +/-1000 -> +/-100
                      le16(p + 4) / 10, le16(p + 6) / 10);
        last_drive_ms = k_uptime_get_32();
        break;

    case CMD_GRIPPER:
        if (len != GRIPPER_PAYLOAD_LEN) break;
        gripper_set(p[0]);
        break;

    case CMD_HOPPER:
        if (len != HOPPER_PAYLOAD_LEN) break;
        if (p[0] == HOPPER_ACT_EXTEND)       hopper_extend();
        else if (p[0] == HOPPER_ACT_RETRACT) hopper_retract();
        else                                 hopper_stop();
        break;

    case CMD_ARM: {
        if (len != ARM_PAYLOAD_LEN) break;
        uint16_t t = leu16(p + 6);
        servo_move(SERVO_ID_SHOULDER,     le16(p)     / 10.0f, t);
        servo_move(SERVO_ID_ELBOW, le16(p + 2) / 10.0f, t);
        servo_move(SERVO_ID_WRIST,    le16(p + 4) / 10.0f, t);
        break;
    }
    }
}

static void uart_output(void)
{
    uint8_t pkt[ODOM_PKT_LEN];
    pkt[0] = PKT_HEADER_0;
    pkt[1] = PKT_HEADER_1;
    pkt[2] = CMD_ODOM;
    pkt[3] = ODOM_PAYLOAD_LEN;

    int32_t counts[4] = {0, 0, 0, 0};   // placeholder until encoders are wired
    memcpy(&pkt[4], counts, ODOM_PAYLOAD_LEN);

    uint8_t chk = CMD_ODOM ^ ODOM_PAYLOAD_LEN;
    for (uint8_t i = 0; i < ODOM_PAYLOAD_LEN; i++) chk ^= pkt[4 + i];
    pkt[ODOM_PKT_LEN - 1] = chk;

    for (int i = 0; i < ODOM_PKT_LEN; i++) {
        uart_poll_out(uart_dev, pkt[i]);
    }
}

void uart_tick(void)
{
    // apply a packet the isr finished parsing
    if (pkt_ready) {
        uint8_t cmd = pkt_cmd;
        uint8_t len = pkt_len;
        uint8_t buf[RX_MAX_PAYLOAD];
        memcpy(buf, pkt_buf, len);
        pkt_ready = false;          // release the slot for the next packet
        apply_packet(cmd, buf, len);
    }

    uint32_t now = k_uptime_get_32();

    // drive watchdog — stop the wheels if no drive command in 500ms
    if ((now - last_drive_ms) >= DRIVE_WATCHDOG_MS) {
        motor_stop_all();
    }

    // odom at 50 Hz
    if ((now - last_odom_ms) >= 20u) {
        last_odom_ms = now;
        uart_output();
    }
}
