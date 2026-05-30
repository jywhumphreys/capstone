#include <zephyr/kernel.h>
#include "comms.h"
#include "motors.h"
#include "gripper.h"

/* Bench bring-up: hold the gripper at one fixed position. Change
 * GRIPPER_POSITION (10 = open .. 60 = closed, clamped to that band), reflash,
 * observe. Set GRIPPER_TEST to 0 for normal UART operation. */
#define GRIPPER_TEST     1
#define GRIPPER_POSITION 10

#if GRIPPER_TEST

int main(void)
{
    if (gripper_init() != 0) {
        printk("gripper_init failed\n");
        return 0;
    }

    gripper_set(GRIPPER_POSITION);
    printk("gripper held at %d\n", GRIPPER_POSITION);
    return 0;
}

#else  /* normal UART-driven operation */

int main(void)
{
    motors_init();
    gripper_init();
    uart_init();

    while (1) {
        uart_tick();
        k_msleep(1);
    }
}

#endif /* GRIPPER_TEST */
