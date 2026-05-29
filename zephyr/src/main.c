#include <zephyr/kernel.h>
#include "comms.h"
#include "motors.h"

/* Bench spin test: drive the motors directly instead of waiting on UART drive
 * commands from the Jetson. Set back to 0 once the UART link is up. */
#define MOTOR_SPIN_TEST 1

#if MOTOR_SPIN_TEST

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

    printk("\n=== mecanum spin test ===\n");

    while (1) {
        /* 1) Each wheel on its own — verify wiring and direction. */
       // for (int m = 0; m < MOTOR_COUNT; m++) {
      //      test_one_motor((motor_id_t)m, 100);
     //   }

        /* 2) Whole-robot motions via inverse kinematics. */
        test_mecanum("forward",      100,    0,    0);
        test_mecanum("backward",    -100,    0,    0);
        test_mecanum("strafe left",    0,  100,    0);
        test_mecanum("strafe right",   0, -100,    0);
        test_mecanum("rotate CCW",     0,    0,  100);
        test_mecanum("rotate CW",      0,    0, -100);

        printk("--- cycle complete, pausing ---\n");
        motor_stop_all();
        k_msleep(2000);
    }
}

#else  /* normal UART-driven operation */

int main(void)
{
    motors_init();
    uart_init();

    while (1) {
        uart_tick();
        k_msleep(1);
    }
}

#endif /* MOTOR_SPIN_TEST */
