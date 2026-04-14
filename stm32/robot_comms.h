#ifndef ROBOT_COMMS_H
#define ROBOT_COMMS_H

#include <stdint.h>
#include "comms_protocol.h"

/* Call once after MX_USART3_UART_Init() */
void robot_comms_init(void);

/* Call from the main loop every ms (uses HAL_GetTick internally).
 * Sends an ODOM packet at 50 Hz and enforces the drive watchdog.        */
void robot_comms_tick(void);

/* Called by HAL_UART_RxCpltCallback — do not call directly */
void robot_comms_uart_rx_callback(void);

/* Drive watchdog timeout: if no CMD_DRIVE received within this many ms,
 * all motors are stopped.                                                */
#define DRIVE_WATCHDOG_MS  500u

#endif /* ROBOT_COMMS_H */
