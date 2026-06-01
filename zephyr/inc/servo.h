#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

// hiwonder HTD-45H bus servos, daisy-chained on usart3 (half-duplex, 115200).
// the three arm joints:
#define SERVO_ID_BASE      1
#define SERVO_ID_SHOULDER  2
#define SERVO_ID_ELBOW     3

// broadcast: every servo obeys, none reply. only use with one servo connected
#define SERVO_ID_BROADCAST 0xFEu

int servo_init(void);   // bring up the bus uart (irq rx). 0 ok, <0 errno

// build + send: 0x55 0x55 id len cmd params... checksum. handles checksum and
// swallows the tx echo. params may be NULL when param_len is 0
void servo_send(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len);

void  servo_move(uint8_t id, float angle_deg, uint16_t time_ms);       // cmd 1
void  servo_move_wait(uint8_t id, float angle_deg, uint16_t time_ms);  // cmd 7
void  servo_move_start(uint8_t id);                                    // cmd 11
void  servo_stop(uint8_t id);                                          // cmd 12
void  servo_set_torque(uint8_t id, bool enable);                       // cmd 31
void  servo_set_id(uint8_t id, uint8_t new_id);                        // cmd 13

// degrees, can be negative. NAN on no/bad response. cmd 28
float servo_read_pos(uint8_t id);

#endif /* SERVO_H */
