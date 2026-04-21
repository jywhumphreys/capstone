#include "comms.h"
#include "comms_protocol.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <string.h>

typedef enum {
    RX_WAIT_H0,
    RX_WAIT_H1,
    RX_WAIT_CMD,
    RX_RECV_DATA,
    RX_WAIT_CHK,
} RxState;

static const struct device *uart_dev;
static RxState  rx_state      = RX_WAIT_H0;
static uint8_t  rx_cmd;
static uint8_t  rx_buf[DRIVE_PAYLOAD_LEN];
static uint8_t  rx_idx;
static uint32_t last_drive_ms = 0;
static uint32_t last_odom_ms  = 0;

static void uart_rx_callback(const struct device *dev, void *user_data);
static void process_drive_packet(const uint8_t *payload);
static void uart_output(void);

void uart_init(void)
{
    //get uart device pointer
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));

    //check if the device is ready for use
    if (!device_is_ready(uart_dev)) {
        return;
    }

    //set the callback function for when data is received and enable the RX interrupt
    uart_irq_callback_user_data_set(uart_dev, uart_rx_callback, NULL);
    uart_irq_rx_enable(uart_dev);
}

static void process_drive_packet(const uint8_t *payload)
{
    /* 4 × int16 big-endian: FL FR RL RR */
    int16_t speeds[4];
    for (int i = 0; i < 4; i++) {
        speeds[i] = (int16_t)((payload[i*2] << 8) | payload[i*2 + 1]);
    }
    // motor_set_all(speeds[0], speeds[1], speeds[2], speeds[3]);
    last_drive_ms = k_uptime_get_32();
}

//uart interrupt handler (ISR)
static void uart_rx_callback(const struct device *dev, void *user_data)
{
    //start processing interrupts
    if (!uart_irq_update(dev)) return;

    //check if uart rx buffer has a received char
    if (!uart_irq_rx_ready(dev)) return;

    //read in data
    uint8_t buffer;
    uart_fifo_read(dev, &buffer, 1);

    switch (rx_state) {
        case RX_WAIT_H0:
            if (buffer == PKT_HEADER_0) rx_state = RX_WAIT_H1;
            break;

        case RX_WAIT_H1:
            rx_state = (buffer == PKT_HEADER_1) ? RX_WAIT_CMD : RX_WAIT_H0;
            break;

        case RX_WAIT_CMD:
            if (buffer == CMD_DRIVE) {
                rx_cmd   = buffer;
                rx_idx   = 0;
                rx_state = RX_RECV_DATA;
            } else {
                rx_state = RX_WAIT_H0;  /* unknown command */
            }
            break;

        case RX_RECV_DATA:
            rx_buf[rx_idx++] = buffer;
            if (rx_idx >= DRIVE_PAYLOAD_LEN)
                rx_state = RX_WAIT_CHK;
            break;

        case RX_WAIT_CHK: {
            uint8_t chk = rx_cmd;
            for (uint8_t i = 0; i < DRIVE_PAYLOAD_LEN; i++) chk ^= rx_buf[i];
            if (chk == buffer)
                process_drive_packet(rx_buf);
            rx_state = RX_WAIT_H0;
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

    /* encoder counts — placeholder zeros until motor_control is ported */
    int32_t counts[4] = {0, 0, 0, 0};
    memcpy(&pkt[3], counts, 16);

    uint8_t chk = CMD_ODOM;
    for (uint8_t i = 0; i < ODOM_PAYLOAD_LEN; i++) chk ^= pkt[3 + i];
    pkt[ODOM_PKT_LEN - 1] = chk;

    for (int i = 0; i < ODOM_PKT_LEN; i++) {
        uart_poll_out(uart_dev, pkt[i]);
    }
}

void uart_tick(void)
{
    uint32_t now = k_uptime_get_32();

    /* drive watchdog — stop motors if no command received in 500ms */
    if ((now - last_drive_ms) >= DRIVE_WATCHDOG_MS) {
        // motor_stop_all();
    }

    /* send odom at 50 Hz */
    if ((now - last_odom_ms) >= 20u) {
        last_odom_ms = now;
        uart_output();
    }
}