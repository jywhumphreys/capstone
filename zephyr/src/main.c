#include <zephyr/kernel.h>
#include "comms.h"
#include "motors.h"
#include "gripper.h"
#include "hopper.h"

/* Bench test selector — set TEST_MODE to one of:
 *   TEST_NONE     normal UART-driven operation
 *   TEST_DRIVE    spin each wheel in turn, then run mecanum motions
 *   TEST_GRIPPER  sweep the gripper open <-> closed within its limits
 *   TEST_HOPPER   extend / retract the linear actuator, looping
 */
#define TEST_NONE     0
#define TEST_DRIVE    1
#define TEST_GRIPPER  2
#define TEST_HOPPER   3

#define TEST_MODE     TEST_DRIVE

#define DRIVE_SPEED            100    /* TEST_DRIVE: wheel speed, 0..100         */
#define HOPPER_TEST_TRAVEL_MS 3000    /* TEST_HOPPER: drive time per direction   */
#define HOPPER_TEST_PAUSE_MS  2000    /* TEST_HOPPER: pause between moves         */

#if TEST_MODE == TEST_DRIVE

static const char *const motor_name[MOTOR_COUNT] = { "FL", "FR", "RL", "RR" };

/* Ramp one wheel up to +peak and back, then down to -peak and back, so you
 * can watch which wheel moves and confirm its direction matches the label. */
static void test_one_motor(motor_id_t m, int peak)
{
    printk("motor %s: forward\n", motor_name[m]);
    for (int s = 0; s <= peak; s += 5) {
        motor_set(m, s);
        k_msleep(50);
    }
    k_msleep(800);
    motor_set(m, 0);
    k_msleep(500);

    printk("motor %s: reverse\n", motor_name[m]);
    for (int s = 0; s >= -peak; s -= 5) {
        motor_set(m, s);
        k_msleep(50);
    }
    k_msleep(800);
    motor_set(m, 0);
    k_msleep(500);
}

/* Run one whole-robot motion through the inverse kinematics, then stop. */
static void test_mecanum(const char *label, int vx, int vy, int omega)
{
    printk("mecanum: %s\n", label);
    mecanum_drive(vx, vy, omega);
    k_msleep(1500);
    motor_stop_all();
    k_msleep(700);
}

int main(void)
{
    if (motors_init() != 0) {
        printk("motors_init failed\n");
        return 0;
    }

    printk("\n=== drive test ===\n");

    while (1) {
        /* 1) Each wheel on its own — verify wiring and direction. */
        for (int m = 0; m < MOTOR_COUNT; m++) {
            test_one_motor((motor_id_t)m, DRIVE_SPEED);
        }

        /* 2) Whole-robot motions via inverse kinematics. */
        test_mecanum("forward",       DRIVE_SPEED,            0,            0);
        test_mecanum("backward",     -DRIVE_SPEED,            0,            0);
        test_mecanum("strafe left",            0,  DRIVE_SPEED,            0);
        test_mecanum("strafe right",           0, -DRIVE_SPEED,            0);
        test_mecanum("rotate CCW",             0,            0,  DRIVE_SPEED);
        test_mecanum("rotate CW",              0,            0, -DRIVE_SPEED);

        printk("--- cycle complete, pausing ---\n");
        motor_stop_all();
        k_msleep(2000);
    }
}

#elif TEST_MODE == TEST_GRIPPER

int main(void)
{
    if (gripper_init() != 0) {
        printk("gripper_init failed\n");
        return 0;
    }

    printk("\n=== gripper test ===\n");

    while (1) {
        gripper_test();
    }
}

#elif TEST_MODE == TEST_HOPPER

int main(void)
{
    if (hopper_init() != 0) {
        printk("hopper_init failed\n");
        return 0;
    }

    printk("\n=== hopper test ===\n");

    while (1) {
        printk("hopper: extend\n");
        hopper_extend();
        k_msleep(HOPPER_TEST_TRAVEL_MS);
        hopper_stop();
        k_msleep(HOPPER_TEST_PAUSE_MS);

        printk("hopper: retract\n");
        hopper_retract();
        k_msleep(HOPPER_TEST_TRAVEL_MS);
        hopper_stop();
        k_msleep(HOPPER_TEST_PAUSE_MS);
    }
}

#else  /* TEST_NONE — normal UART-driven operation */

int main(void)
{
    motors_init();
    gripper_init();
    hopper_init();
    uart_init();

    while (1) {
        uart_tick();
        hopper_tick();
        k_msleep(1);
    }
}

#endif /* TEST_MODE */
