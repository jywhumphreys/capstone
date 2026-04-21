#ifndef COMMS_PROTOCOL_H
#define COMMS_PROTOCOL_H

#include <stdint.h>

/* ── Framing ─────────────────────────────────────────────────────────────── */
#define PKT_HEADER_0    0xAAu
#define PKT_HEADER_1    0x55u

/* ── Command IDs ─────────────────────────────────────────────────────────── */
#define CMD_DRIVE       0x01u   /* Jetson → STM32 */
#define CMD_ODOM        0x02u   /* STM32  → Jetson */

/* ── Packet layouts ──────────────────────────────────────────────────────── */
/*
 * DRIVE packet (12 bytes, Jetson → STM32):
 *   [0xAA][0x55][0x01][FL_H][FL_L][FR_H][FR_L][RL_H][RL_L][RR_H][RR_L][CHK]
 *   Wheel speeds: int16, big-endian, range –1000 … +1000
 *   (+1000 = full forward, –1000 = full reverse)
 *
 * ODOM packet (20 bytes, STM32 → Jetson, sent at 50 Hz):
 *   [0xAA][0x55][0x02][FL×4][FR×4][RL×4][RR×4][CHK]
 *   Encoder ticks: int32, little-endian, cumulative since boot
 *
 * CHK = XOR of every byte from CMD through last payload byte (inclusive).
 * Motor order: 0=FL, 1=FR, 2=RL, 3=RR
 */

#define DRIVE_PAYLOAD_LEN   8u   /* 4 × int16 */
#define ODOM_PAYLOAD_LEN    16u  /* 4 × int32 */

#define DRIVE_PKT_LEN   (2u + 1u + DRIVE_PAYLOAD_LEN + 1u)   /* 12 */
#define ODOM_PKT_LEN    (2u + 1u + ODOM_PAYLOAD_LEN  + 1u)   /* 20 */

/* Speed clamp helpers */
#define SPEED_MAX   1000
#define SPEED_MIN  -1000

#endif /* COMMS_PROTOCOL_H */
