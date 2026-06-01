#include "servo.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <math.h>

#define SERVO_UART_NODE DT_NODELABEL(usart3)

#define SERVO_HEADER  0x55u

#define CMD_MOVE         1u
#define CMD_MOVE_WAIT    7u
#define CMD_MOVE_START   11u
#define CMD_MOVE_STOP    12u
#define CMD_SET_ID       13u
#define CMD_POS_READ     28u
#define CMD_LOAD_UNLOAD  31u

// 0..240 deg <-> 0..1000 ticks
#define DEG_TO_TICKS(d)  ((int)((d) * 1000.0f / 240.0f))
#define TICKS_TO_DEG(t)  ((t) * 240.0f / 1000.0f)

#define RX_BYTE_TIMEOUT_MS  50   // one byte at 115200 is ~87us; generous

static const struct device *const uart = DEVICE_DT_GET(SERVO_UART_NODE);

// isr fills, thread drains (single producer / single consumer)
RING_BUF_DECLARE(servo_rx_rb, 64);

static void servo_rx_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }
    uint8_t b;
    while (uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &b, 1) == 1) {
            ring_buf_put(&servo_rx_rb, &b, 1);
        }
    }
}

int servo_init(void)
{
    if (!device_is_ready(uart)) {
        return -ENODEV;
    }

    uart_irq_rx_disable(uart);
    uart_irq_callback_user_data_set(uart, servo_rx_isr, NULL);
    uart_irq_rx_enable(uart);
    return 0;
}

// one byte from the rx ring buffer, waiting up to RX_BYTE_TIMEOUT_MS
static int rx_byte(uint8_t *b)
{
    uint32_t start = k_uptime_get_32();

    do {
        if (ring_buf_get(&servo_rx_rb, b, 1) == 1) {
            return 0;
        }
        k_busy_wait(100);
    } while ((k_uptime_get_32() - start) < RX_BYTE_TIMEOUT_MS);

    return -ETIMEDOUT;
}

void servo_send(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    uint8_t buf[16];
    uint8_t len = param_len + 3u;   // len = num_params + 3

    buf[0] = SERVO_HEADER;
    buf[1] = SERVO_HEADER;
    buf[2] = id;
    buf[3] = len;
    buf[4] = cmd;
    for (uint8_t i = 0; i < param_len; i++) {
        buf[5 + i] = params[i];
    }

    // checksum = ~(id + len + cmd + sum(params))
    uint16_t sum = 0;
    for (uint8_t i = 2; i < (uint8_t)(len + 2u); i++) {
        sum += buf[i];
    }
    buf[5 + param_len] = (uint8_t)(~sum);

    uint8_t total = param_len + 6u;   // header(2) + id + len + cmd + params + chk

    // half-duplex: clear rx, send, then swallow our own echo
    ring_buf_reset(&servo_rx_rb);
    for (uint8_t i = 0; i < total; i++) {
        uart_poll_out(uart, buf[i]);
    }
    uint8_t echo;
    for (uint8_t i = 0; i < total; i++) {
        (void)rx_byte(&echo);
    }
}

// read a response into params[]. returns num params, or <0 on timeout/bad frame
static int servo_read_response(uint8_t *params, uint8_t max_params)
{
    uint8_t b, id, len, cmd;
    int sync = 0;

    // sync on 0x55 0x55
    while (sync < 2) {
        if (rx_byte(&b) != 0) {
            return -ETIMEDOUT;
        }
        sync = (b == SERVO_HEADER) ? sync + 1 : 0;
    }

    if (rx_byte(&id) || rx_byte(&len) || rx_byte(&cmd)) {
        return -ETIMEDOUT;
    }
    if (len < 3u) {
        return -EIO;
    }

    uint8_t nparams = len - 3u;
    if (nparams > max_params) {
        return -EIO;
    }
    for (uint8_t i = 0; i < nparams; i++) {
        if (rx_byte(&params[i])) {
            return -ETIMEDOUT;
        }
    }

    uint8_t chk;
    if (rx_byte(&chk)) {
        return -ETIMEDOUT;
    }

    uint16_t sum = id + len + cmd;
    for (uint8_t i = 0; i < nparams; i++) {
        sum += params[i];
    }
    if ((uint8_t)(~sum) != chk) {
        return -EIO;
    }
    return nparams;
}

void servo_move(uint8_t id, float angle_deg, uint16_t time_ms)
{
    int ticks = DEG_TO_TICKS(angle_deg);
    uint8_t p[4] = {
        (uint8_t)(ticks & 0xFF),
        (uint8_t)((ticks >> 8) & 0xFF),
        (uint8_t)(time_ms & 0xFF),
        (uint8_t)((time_ms >> 8) & 0xFF),
    };
    servo_send(id, CMD_MOVE, p, 4);
}

void servo_move_wait(uint8_t id, float angle_deg, uint16_t time_ms)
{
    int ticks = DEG_TO_TICKS(angle_deg);
    uint8_t p[4] = {
        (uint8_t)(ticks & 0xFF),
        (uint8_t)((ticks >> 8) & 0xFF),
        (uint8_t)(time_ms & 0xFF),
        (uint8_t)((time_ms >> 8) & 0xFF),
    };
    servo_send(id, CMD_MOVE_WAIT, p, 4);
}

void servo_move_start(uint8_t id)
{
    servo_send(id, CMD_MOVE_START, NULL, 0);
}

void servo_stop(uint8_t id)
{
    servo_send(id, CMD_MOVE_STOP, NULL, 0);
}

void servo_set_torque(uint8_t id, bool enable)
{
    uint8_t p = enable ? 1u : 0u;
    servo_send(id, CMD_LOAD_UNLOAD, &p, 1);
}

void servo_set_id(uint8_t id, uint8_t new_id)
{
    servo_send(id, CMD_SET_ID, &new_id, 1);
}

float servo_read_pos(uint8_t id)
{
    servo_send(id, CMD_POS_READ, NULL, 0);

    uint8_t p[4];
    int n = servo_read_response(p, sizeof(p));
    if (n < 2) {
        return NAN;
    }

    int16_t ticks = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    return TICKS_TO_DEG(ticks);
}
