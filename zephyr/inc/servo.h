#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

/* Hiwonder HTD-45H serial bus servos, daisy-chained on USART3 (half-duplex,
 * 115200 baud). Three servos make up the arm: */
#define SERVO_ID_BASE      1
#define SERVO_ID_SHOULDER  2
#define SERVO_ID_ELBOW     3

/* Broadcast address — every servo on the bus obeys it (and none reply). Use
 * only with a single servo connected, e.g. when assigning IDs. */
#define SERVO_ID_BROADCAST 0xFEu

/* Initialise the servo bus UART (interrupt RX for echo/response capture).
 * Returns 0 on success, negative errno if the device is not ready. */
int servo_init(void);

/* Build and transmit one packet: 0x55 0x55 ID LEN CMD PARAMS... CHECKSUM.
 * Handles the checksum and discards the TX echo seen on the shared wire.
 * params may be NULL when param_len is 0. */
void servo_send(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len);

void  servo_move(uint8_t id, float angle_deg, uint16_t time_ms);       /* CMD 1  */
void  servo_move_wait(uint8_t id, float angle_deg, uint16_t time_ms);  /* CMD 7  */
void  servo_move_start(uint8_t id);                                    /* CMD 11 */
void  servo_stop(uint8_t id);                                          /* CMD 12 */
void  servo_set_torque(uint8_t id, bool enable);                       /* CMD 31 */

/* Assign a new bus ID. Address `id` (the servo's current ID, or
 * SERVO_ID_BROADCAST with exactly one servo connected). CMD 13. */
void  servo_set_id(uint8_t id, uint8_t new_id);

/* Read current position. Returns degrees (can be negative); NAN on no/bad
 * response. CMD 28. */
float servo_read_pos(uint8_t id);

#endif /* SERVO_H */
