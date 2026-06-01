#include <zephyr/kernel.h>
#include <math.h>
#include "comms.h"
#include "motors.h"
#include "gripper.h"
#include "hopper.h"
#include "servo.h"

// pick a test mode (or TEST_NONE for normal uart operation)
#define TEST_NONE     0   // uart-driven
#define TEST_DRIVE    1   // mecanum demo
#define TEST_GRIPPER  2   // gripper open/close
#define TEST_HOPPER   3   // actuator extend/retract
#define TEST_ARM      4   // arm servos, ping-pong 1-2-3-2-1
#define TEST_SET_ID   5   // assign an id to a single connected servo
#define TEST_SWAP_ID  6   // swap two ids on the chain
#define TEST_DEMO     7   // all subsystems in sequence

#define TEST_MODE     TEST_DEMO

// TEST_SET_ID: id to assign. connect exactly one servo, flash, repeat per servo
#define SET_ID_TARGET  SERVO_ID_ELBOW

// TEST_SWAP_ID: swap these two, routed through a temp id so no collision
#define SWAP_ID_A      SERVO_ID_SHOULDER
#define SWAP_ID_B      SERVO_ID_ELBOW
#define SWAP_ID_TEMP   10

#define DRIVE_SPEED            100    // TEST_DRIVE wheel speed 0..100
#define HOPPER_TEST_TRAVEL_MS 3000    // TEST_HOPPER drive time per direction
#define HOPPER_TEST_PAUSE_MS  2000

#if TEST_MODE == TEST_DRIVE

// run one mecanum motion for ms, then stop
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
        // cardinals
        demo("forward",         S,  0,    0, 1200);
        demo("backward",       -S,  0,    0, 1200);
        demo("strafe left",     0,  S,    0, 1200);
        demo("strafe right",    0, -S,    0, 1200);

        // diagonals (translate at 45 deg, no rotation)
        demo("diag fwd-right",  S, -S,    0, 1000);
        demo("diag back-left", -S,  S,    0, 1000);
        demo("diag fwd-left",   S,  S,    0, 1000);
        demo("diag back-right",-S, -S,    0, 1000);

        // rotate in place
        demo("spin CCW",        0,  0,    S, 1500);
        demo("spin CW",         0,  0,   -S, 1500);

        // drive + turn at once
        demo("arc fwd + CCW",   S,  0,  S/2, 1500);
        demo("arc fwd + CW",    S,  0, -S/2, 1500);

        // strafe a square without turning the body
        printk("-- holonomic square --\n");
        demo("  side 1 (fwd)",   S,  0,  0, 900);
        demo("  side 2 (right)", 0, -S,  0, 900);
        demo("  side 3 (back)", -S,  0,  0, 900);
        demo("  side 4 (left)",  0,  S,  0, 900);

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
        gripper_set(100);          // clamps to closed limit
        k_msleep(10000);

        printk("gripper: open\n");
        gripper_set(0);            // clamps to open limit
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

// cycle all three in a ping-pong order, each alternating between two angles.
// 30..210 leaves ~30 deg margin off the hard stops — narrow if mounted tight
#define ARM_TEST_A_DEG    30.0f
#define ARM_TEST_B_DEG    210.0f
#define ARM_TEST_TIME_MS  1000
#define ARM_TEST_PAUSE_MS 500

static void arm_report(uint8_t id)
{
    float deg = servo_read_pos(id);
    if (isnan(deg)) {
        printk("servo %u: no response\n", id);
    } else {
        int tenths = (int)(deg * 10.0f);   // %f isn't enabled
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

    printk("\n=== arm test (1 -> 2 -> 3 -> 2 -> 1 ...) ===\n");

    servo_set_torque(SERVO_ID_BASE, true);
    servo_set_torque(SERVO_ID_SHOULDER, true);
    servo_set_torque(SERVO_ID_ELBOW, true);

    // looping this gives 1,2,3,2,1,2,3,2,...
    static const uint8_t order[] = {
        SERVO_ID_BASE, SERVO_ID_SHOULDER, SERVO_ID_ELBOW, SERVO_ID_SHOULDER
    };
    bool at_b[SERVO_ID_ELBOW + 1] = { false };

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

    printk("\n=== set servo id -> %d (one servo connected) ===\n", SET_ID_TARGET);

    // broadcast works at any current id, but hits every servo — hence one at a time
    servo_set_id(SERVO_ID_BROADCAST, SET_ID_TARGET);
    k_msleep(100);

    // confirm it answers at the new id
    float deg = servo_read_pos(SET_ID_TARGET);
    if (isnan(deg)) {
        printk("readback failed at id %d — exactly one servo connected?\n",
               SET_ID_TARGET);
    } else {
        printk("ok: servo answers at id %d (%d deg)\n", SET_ID_TARGET, (int)deg);
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

    printk("\n=== swap servo ids %d <-> %d ===\n", SWAP_ID_A, SWAP_ID_B);

    // route through a temp id so no two servos ever share an address
    servo_set_id(SWAP_ID_A, SWAP_ID_TEMP);
    k_msleep(100);
    servo_set_id(SWAP_ID_B, SWAP_ID_A);
    k_msleep(100);
    servo_set_id(SWAP_ID_TEMP, SWAP_ID_B);
    k_msleep(100);

    printk("id %d: %s\n", SWAP_ID_A,
           isnan(servo_read_pos(SWAP_ID_A)) ? "no response" : "ok");
    printk("id %d: %s\n", SWAP_ID_B,
           isnan(servo_read_pos(SWAP_ID_B)) ? "no response" : "ok");

    while (1) {
        k_msleep(1000);
    }
}

#elif TEST_MODE == TEST_DEMO

// self-contained showcase: drive -> gripper -> hopper -> arm, looping. one
// subsystem at a time so peak power stays bounded. no jetson needed
int main(void)
{
    motors_init();
    gripper_init();
    hopper_init();
    servo_init();

    const int S = 80;   // a touch under full for control

    printk("\n=== robot demo ===\n");

    servo_set_torque(SERVO_ID_BASE, true);
    servo_set_torque(SERVO_ID_SHOULDER, true);
    servo_set_torque(SERVO_ID_ELBOW, true);

    while (1) {
        // drive — a sampler of mecanum moves
        printk("[drive] forward\n");
        mecanum_drive(S, 0, 0);   k_msleep(1200); motor_stop_all(); k_msleep(400);
        printk("[drive] strafe right\n");
        mecanum_drive(0, -S, 0);  k_msleep(1200); motor_stop_all(); k_msleep(400);
        printk("[drive] diagonal\n");
        mecanum_drive(S, S, 0);   k_msleep(1000); motor_stop_all(); k_msleep(400);
        printk("[drive] spin\n");
        mecanum_drive(0, 0, S);   k_msleep(1500); motor_stop_all(); k_msleep(600);

        // gripper
        printk("[gripper] open\n");
        gripper_set(0);    k_msleep(1200);
        printk("[gripper] close\n");
        gripper_set(100);  k_msleep(1200);

        // hopper
        printk("[hopper] extend\n");
        hopper_extend();   k_msleep(3000); hopper_stop(); k_msleep(800);
        printk("[hopper] retract\n");
        hopper_retract();  k_msleep(3000); hopper_stop(); k_msleep(800);

        // arm — home, then a little elbow wave
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

#else  // TEST_NONE — normal uart operation

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

#endif
