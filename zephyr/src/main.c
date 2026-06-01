#include <zephyr/kernel.h>
#include <math.h>
#include "comms.h"
#include "motors.h"
#include "gripper.h"
#include "hopper.h"
#include "servo.h"

/* Bench test selector — set TEST_MODE to one of:
 *   TEST_NONE     normal UART-driven operation
 *   TEST_DRIVE    spin each wheel in turn, then run mecanum motions
 *   TEST_GRIPPER  sweep the gripper open <-> closed within its limits
 *   TEST_HOPPER   extend / retract the linear actuator, looping
 *   TEST_ARM      cycle the 3 arm servos in a ping-pong order
 *   TEST_SET_ID   assign a bus ID to a single connected servo
 *   TEST_SWAP_ID  swap two servo IDs on the live chain (via a temp ID)
 *   TEST_DEMO     run all four subsystems in sequence (self-contained demo)
 */
#define TEST_NONE     0
#define TEST_DRIVE    1
#define TEST_GRIPPER  2
#define TEST_HOPPER   3
#define TEST_ARM      4
#define TEST_SET_ID   5
#define TEST_SWAP_ID  6
#define TEST_DEMO     7

#define TEST_MODE     TEST_DEMO

/* TEST_SET_ID: the ID to assign. Connect EXACTLY ONE servo, set this to 1
 * (base), flash, confirm; then connect only the next servo, set to 2, etc. */
#define SET_ID_TARGET  SERVO_ID_ELBOW

/* TEST_SWAP_ID: swap two IDs on the chained bus, addressing each by its
 * current ID and routing through SWAP_ID_TEMP so there's never a collision. */
#define SWAP_ID_A      SERVO_ID_SHOULDER   /* 2 */
#define SWAP_ID_B      SERVO_ID_ELBOW      /* 3 */
#define SWAP_ID_TEMP   10                  /* any unused ID */

#define DRIVE_SPEED            100    /* TEST_DRIVE: wheel speed, 0..100         */
#define HOPPER_TEST_TRAVEL_MS 3000    /* TEST_HOPPER: drive time per direction   */
#define HOPPER_TEST_PAUSE_MS  2000    /* TEST_HOPPER: pause between moves         */

#if TEST_MODE == TEST_DRIVE

/* Run one mecanum motion for `ms`, then stop and settle briefly. */
static void demo(const char *label, int vx, int vy, int omega, int ms)
{
    printk("%s\n", label);
    mecanum_drive(vx, vy, omega);
    k_msleep(ms);
    motor_stop_all();
    k_msleep(400);
}

int main(void)
{
    if (motors_init() != 0) {
        printk("motors_init failed\n");
        return 0;
    }

    const int S = DRIVE_SPEED;

    printk("\n=== mecanum demo ===\n");

    while (1) {
        /* Cardinal translation. */
        demo("forward",         S,  0,    0, 1200);
        demo("backward",       -S,  0,    0, 1200);
        demo("strafe left",     0,  S,    0, 1200);
        demo("strafe right",    0, -S,    0, 1200);

        /* Diagonals — translate at 45 deg without rotating. */
        demo("diag fwd-right",  S, -S,    0, 1000);
        demo("diag back-left", -S,  S,    0, 1000);
        demo("diag fwd-left",   S,  S,    0, 1000);
        demo("diag back-right",-S, -S,    0, 1000);

        /* Rotate in place. */
        demo("spin CCW",        0,  0,    S, 1500);
        demo("spin CW",         0,  0,   -S, 1500);

        /* Combined: drive and turn at once (arcs). */
        demo("arc fwd + CCW",   S,  0,  S/2, 1500);
        demo("arc fwd + CW",    S,  0, -S/2, 1500);

        /* Holonomic square — strafe each side, body never turns. */
        printk("-- holonomic square --\n");
        demo("  side 1 (fwd)",   S,  0,  0, 900);
        demo("  side 2 (right)", 0, -S,  0, 900);
        demo("  side 3 (back)", -S,  0,  0, 900);
        demo("  side 4 (left)",  0,  S,  0, 900);

        /* Finale: full-speed pirouette. */
        demo("pirouette",        0,  0,  S, 2500);

        printk("--- demo complete, pausing ---\n");
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

    printk("\n=== gripper test (close, hold 10s, open) ===\n");

    while (1) {
        printk("gripper: close\n");
        gripper_set(100);          /* clamps to the closed limit */
        k_msleep(10000);

        printk("gripper: open\n");
        gripper_set(0);            /* clamps to the open limit */
        k_msleep(2000);
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

#elif TEST_MODE == TEST_ARM

/* Arm test: cycle all three servos in a ping-pong order (1->2->3->2->1->...)
 * to confirm each responds in turn. Each servo alternates between two angles
 * when its turn comes. 30..210 deg is a wide, visible sweep with ~30 deg
 * margin from the 0/240 hard stops — narrow these if a joint is mounted with
 * tighter mechanical limits. */
#define ARM_TEST_A_DEG    30.0f
#define ARM_TEST_B_DEG    210.0f
#define ARM_TEST_TIME_MS  1000     /* commanded move duration */
#define ARM_TEST_PAUSE_MS 500      /* settle time after each move */

static void arm_report(uint8_t id)
{
    float deg = servo_read_pos(id);
    if (isnan(deg)) {
        printk("servo %u: no response\n", id);
    } else {
        int tenths = (int)(deg * 10.0f);   /* %f isn't enabled */
        printk("servo %u: %d.%d deg\n", id,
               tenths / 10, (tenths < 0 ? -tenths : tenths) % 10);
    }
}

int main(void)
{
    if (servo_init() != 0) {
        printk("servo_init failed\n");
        return 0;
    }

    printk("\n=== arm test (servos 1 -> 2 -> 3 -> 2 -> 1 ...) ===\n");

    servo_set_torque(SERVO_ID_BASE, true);
    servo_set_torque(SERVO_ID_SHOULDER, true);
    servo_set_torque(SERVO_ID_ELBOW, true);

    /* Looping this order yields the ping-pong 1,2,3,2,1,2,3,2,... */
    static const uint8_t order[] = {
        SERVO_ID_BASE, SERVO_ID_SHOULDER, SERVO_ID_ELBOW, SERVO_ID_SHOULDER
    };
    bool at_b[SERVO_ID_ELBOW + 1] = { false };   /* per-id target toggle */

    while (1) {
        for (int i = 0; i < (int)ARRAY_SIZE(order); i++) {
            uint8_t id = order[i];
            float target = at_b[id] ? ARM_TEST_A_DEG : ARM_TEST_B_DEG;
            at_b[id] = !at_b[id];

            printk("-> servo %u\n", id);
            servo_move(id, target, ARM_TEST_TIME_MS);
            k_msleep(ARM_TEST_TIME_MS + ARM_TEST_PAUSE_MS);
            arm_report(id);
        }
    }
}

#elif TEST_MODE == TEST_SET_ID

int main(void)
{
    if (servo_init() != 0) {
        printk("servo_init failed\n");
        return 0;
    }

    printk("\n=== set servo ID -> %d ===\n", SET_ID_TARGET);
    printk("Make sure EXACTLY ONE servo is connected.\n");

    /* Broadcast works regardless of the servo's current (possibly default) ID,
     * but addresses every servo on the bus — hence the one-at-a-time rule. */
    servo_set_id(SERVO_ID_BROADCAST, SET_ID_TARGET);
    k_msleep(100);

    /* Confirm: the servo should now answer at its new ID. */
    float deg = servo_read_pos(SET_ID_TARGET);
    if (isnan(deg)) {
        printk("readback FAILED at ID %d — is exactly one servo connected?\n",
               SET_ID_TARGET);
    } else {
        printk("OK: servo now responds at ID %d (%d deg)\n",
               SET_ID_TARGET, (int)deg);
    }

    while (1) {
        k_msleep(1000);
    }
}

#elif TEST_MODE == TEST_SWAP_ID

int main(void)
{
    if (servo_init() != 0) {
        printk("servo_init failed\n");
        return 0;
    }

    printk("\n=== swap servo IDs %d <-> %d (chain connected) ===\n",
           SWAP_ID_A, SWAP_ID_B);

    /* Route through a temp ID so no two servos ever share an address. */
    servo_set_id(SWAP_ID_A, SWAP_ID_TEMP);   /* A -> temp */
    k_msleep(100);
    servo_set_id(SWAP_ID_B, SWAP_ID_A);      /* B -> A    */
    k_msleep(100);
    servo_set_id(SWAP_ID_TEMP, SWAP_ID_B);   /* temp -> B */
    k_msleep(100);

    /* Confirm both new IDs answer. */
    printk("ID %d: %s\n", SWAP_ID_A,
           isnan(servo_read_pos(SWAP_ID_A)) ? "no response" : "OK");
    printk("ID %d: %s\n", SWAP_ID_B,
           isnan(servo_read_pos(SWAP_ID_B)) ? "no response" : "OK");

    while (1) {
        k_msleep(1000);
    }
}

#elif TEST_MODE == TEST_DEMO

/* Self-contained showcase: drive -> gripper -> hopper -> arm, in sequence,
 * looping. One subsystem actuates at a time, so peak power stays bounded.
 * No Jetson/UART needed. */
int main(void)
{
    motors_init();
    gripper_init();
    hopper_init();
    servo_init();

    const int S = 80;   /* drive speed — a touch under full for control */

    printk("\n=== robot demo ===\n");

    /* Arm joints to a neutral pose once, up front. */
    servo_set_torque(SERVO_ID_BASE, true);
    servo_set_torque(SERVO_ID_SHOULDER, true);
    servo_set_torque(SERVO_ID_ELBOW, true);

    while (1) {
        /* --- drive: a sampler of mecanum moves --- */
        printk("[drive] forward\n");
        mecanum_drive(S, 0, 0);   k_msleep(1200); motor_stop_all(); k_msleep(400);
        printk("[drive] strafe right\n");
        mecanum_drive(0, -S, 0);  k_msleep(1200); motor_stop_all(); k_msleep(400);
        printk("[drive] diagonal\n");
        mecanum_drive(S, S, 0);   k_msleep(1000); motor_stop_all(); k_msleep(400);
        printk("[drive] spin\n");
        mecanum_drive(0, 0, S);   k_msleep(1500); motor_stop_all(); k_msleep(600);

        /* --- gripper --- */
        printk("[gripper] open\n");
        gripper_set(0);    k_msleep(1200);
        printk("[gripper] close\n");
        gripper_set(100);  k_msleep(1200);

        /* --- hopper --- */
        printk("[hopper] extend\n");
        hopper_extend();   k_msleep(3000); hopper_stop(); k_msleep(800);
        printk("[hopper] retract\n");
        hopper_retract();  k_msleep(3000); hopper_stop(); k_msleep(800);

        /* --- arm: home, then a little elbow wave --- */
        printk("[arm] pose + wave\n");
        servo_move(SERVO_ID_BASE,     120.0f, 800);
        servo_move(SERVO_ID_SHOULDER, 120.0f, 800);
        servo_move(SERVO_ID_ELBOW,    120.0f, 800);
        k_msleep(1000);
        servo_move(SERVO_ID_ELBOW,     80.0f, 600); k_msleep(800);
        servo_move(SERVO_ID_ELBOW,    160.0f, 600); k_msleep(800);
        servo_move(SERVO_ID_ELBOW,    120.0f, 600); k_msleep(800);

        printk("--- demo complete, pausing ---\n");
        k_msleep(3000);
    }
}

#else  /* TEST_NONE — normal UART-driven operation */

int main(void)
{
    motors_init();
    gripper_init();
    hopper_init();
    servo_init();
    uart_init();

    while (1) {
        uart_tick();
        hopper_tick();
        k_msleep(1);
    }
}

#endif /* TEST_MODE */
