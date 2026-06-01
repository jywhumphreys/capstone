#ifndef COMMS_PROTOCOL_H
#define COMMS_PROTOCOL_H

#include <stdint.h>

// uart frame: [0xAA][0x55][cmd][len][payload... len bytes][chk]
// chk = xor of cmd, len, and every payload byte. multi-byte fields little-endian

#define PKT_HEADER_0  0xAAu
#define PKT_HEADER_1  0x55u

// jetson -> stm32
#define CMD_DRIVE     0x01u   // 8B: 4x int16 wheel speed FL FR RL RR, +/-1000
#define CMD_GRIPPER   0x03u   // 1B: position 0..100
#define CMD_HOPPER    0x04u   // 1B: 0 stop / 1 extend / 2 retract
#define CMD_ARM       0x05u   // 8B: 3x int16 angle (tenths-deg) + uint16 time_ms

// stm32 -> jetson
#define CMD_ODOM      0x02u   // 16B: 4x int32 encoder ticks

#define DRIVE_PAYLOAD_LEN    8u
#define GRIPPER_PAYLOAD_LEN  1u
#define HOPPER_PAYLOAD_LEN   1u
#define ARM_PAYLOAD_LEN      8u
#define ODOM_PAYLOAD_LEN     16u

#define RX_MAX_PAYLOAD       16u
#define ODOM_PKT_LEN         (2u + 1u + 1u + ODOM_PAYLOAD_LEN + 1u)   // 21

// hopper actions
#define HOPPER_ACT_STOP     0u
#define HOPPER_ACT_EXTEND   1u
#define HOPPER_ACT_RETRACT  2u

// drive speed is +/-1000 on the wire, +/-100 in the firmware
#define DRIVE_SPEED_FULL    1000

#endif /* COMMS_PROTOCOL_H */
