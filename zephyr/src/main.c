#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <math.h>
#include <stdlib.h>
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
#define TEST_ARM      4   // interactive: set arm angles over serial
#define TEST_SET_ID   5   // assign an id to a single connected servo
#define TEST_SWAP_ID  6   // swap two ids on the chain
#define TEST_DEMO     7   // full pick-and-drop + hopper routine

#define TEST_MODE     TEST_NONE

// TEST_SET_ID: id to assign. connect exactly one servo, flash, repeat per servo
#define SET_ID_TARGET  SERVO_ID_WRIST

// TEST_SWAP_ID: swap these two, routed through a temp id so no collision
#define SWAP_ID_A      SERVO_ID_ELBOW
#define SWAP_ID_B      SERVO_ID_WRIST
#define SWAP_ID_TEMP   10

#define DRIVE_SPEED            100    // TEST_DRIVE wheel speed 0..100
#define HOPPER_TEST_TRAVEL_MS 3000    // hopper drive time per direction
#define HOPPER_TEST_PAUSE_MS  2000

#if TEST_MODE == TEST_DRIVE

int main(void)
{
    if (motors_init() != 0) {
        printk("motors_init failed\n");
        return 0;
    }

    printk("drive: backward 2s\n");
    mecanum_drive(-DRIVE_SPEED, 0, 0);
    k_msleep(2000);
    motor_stop_all();
    return 0;
}

#elif TEST_MODE == TEST_GRIPPER

#define OPEN_POSITION 10   // open
#define GRAB_POSITION 30   // grab (closed)

int main(void)
{
    if (gripper_init() != 0) {
        printk("gripper_init failed\n");
        return 0;
    }

    printk("gripper: open\n");
    gripper_set(OPEN_POSITION);
    k_msleep(1500);

    printk("gripper: grab at %d\n", GRAB_POSITION);
    gripper_set(GRAB_POSITION);
    return 0;
}

#elif TEST_MODE == TEST_HOPPER

int main(void)
{
    if (hopper_init() != 0) {
        printk("hopper_init failed\n");
        return 0;
    }

    printk("hopper: lower (retract)\n");
    hopper_retract();
    k_msleep(HOPPER_TEST_TRAVEL_MS);
    hopper_stop();
    return 0;
}

#elif TEST_MODE == TEST_ARM

// interactive arm angle setter over the serial console. starts all three at
// center (120), then type "<id 1-3> <deg>" (e.g. "2 124.5") to move one joint,
// or "c" to recenter. moves are slow so a fat-finger won't slam the arm.
static const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define SET_MOVE_MS 800

static void read_line(char *buf, int max)
{
    int i = 0;
    while (1) {
        uint8_t c;
        if (uart_poll_in(cons, &c) == 0) {
            if (c == '\r' || c == '\n') {
                if (i > 0) { buf[i] = '\0'; return; }
            } else if ((c == 0x08 || c == 0x7f) && i > 0) {
                i--;                          // backspace
            } else if (i < max - 1) {
                buf[i++] = c;
            }
        } else {
            k_msleep(2);
        }
    }
}

static void arm_goto(uint8_t id, float deg)
{
    servo_move(id, deg, SET_MOVE_MS);
    int t = (int)(deg * 10.0f);              // %f isn't enabled
    printk("servo %u -> %d.%d deg\n", id, t / 10, (t < 0 ? -t : t) % 10);
}

int main(void)
{
    if (servo_init() != 0) {
        printk("servo_init failed\n");
        return 0;
    }

    servo_set_torque(SERVO_ID_SHOULDER, true);
    servo_set_torque(SERVO_ID_ELBOW, true);
    servo_set_torque(SERVO_ID_WRIST, true);

    arm_goto(SERVO_ID_SHOULDER, 120.0f);     // start from center
    arm_goto(SERVO_ID_ELBOW,    120.0f);
    arm_goto(SERVO_ID_WRIST,    120.0f);

    printk("\narm angle setter — type \"<id 1-3> <deg>\" (e.g. \"2 124.5\"), or \"c\" to center\n");

    char line[32];
    while (1) {
        read_line(line, sizeof(line));

        if (line[0] == 'c') {
            arm_goto(SERVO_ID_SHOULDER, 120.0f);
            arm_goto(SERVO_ID_ELBOW,    120.0f);
            arm_goto(SERVO_ID_WRIST,    120.0f);
            continue;
        }

        char *end;
        long id = strtol(line, &end, 10);
        if (end == line || id < 1 || id > 3) {
            printk("? use \"<id 1-3> <deg>\" or \"c\"\n");
            continue;
        }
        arm_goto((uint8_t)id, (float)strtod(end, NULL));
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

// drive demo — the arm is never touched. strafe out and back to center in all
// 8 directions, tip the hopper, cycle the gripper, then a rotate flourish.
#define DEMO_SPEED       100
#define OUT_MS           800     // strafe out (and back) duration
#define GRIP_OPEN        10
#define GRIP_CLOSED      30
#define GRIP_MS          800
#define SPIN_MS_PER_DEG  25      // at DEMO_SPEED 100 (180 deg = 4500 ms)

// arm start pose — set once, then left untouched. shoulder/elbow/wrist
#define ARM_S  130.0f
#define ARM_E  200.0f
#define ARM_W   90.0f

// drive a vector out for OUT_MS, then the reverse to return to center
static void out_back(int vx, int vy)
{
    mecanum_drive(vx, vy, 0);
    k_msleep(OUT_MS);
    motor_stop_all();
    k_msleep(400);
    mecanum_drive(-vx, -vy, 0);
    k_msleep(OUT_MS);
    motor_stop_all();
    k_msleep(600);
}

// spin in place: dir +1 = ccw, -1 = cw
static void spin_deg(int dir, int deg)
{
    mecanum_drive(0, 0, dir * DEMO_SPEED);
    k_msleep(deg * SPIN_MS_PER_DEG);
    motor_stop_all();
    k_msleep(400);
}

int main(void)
{
    motors_init();
    gripper_init();
    hopper_init();

    const int S = DEMO_SPEED;

    printk("\n=== drive demo ===\n");

    // set the arm to its start pose (then leave it), and close the hopper
    printk("reset: arm + hopper\n");
    servo_init();
    servo_set_torque(SERVO_ID_SHOULDER, true);
    servo_set_torque(SERVO_ID_ELBOW, true);
    servo_set_torque(SERVO_ID_WRIST, true);
    hopper_retract();                       // close the hopper while the arm moves
    servo_move(SERVO_ID_SHOULDER, ARM_S, 1500);
    servo_move(SERVO_ID_ELBOW,    ARM_E, 1500);
    servo_move(SERVO_ID_WRIST,    ARM_W, 1500);
    k_msleep(2000);
    hopper_stop();

    // hold before the demo begins
    printk("starting in 5s...\n");
    k_msleep(5000);

    // 8 directions, each out then back to center
    printk("forward\n");    out_back( S,  0);
    printk("backward\n");   out_back(-S,  0);
    printk("left\n");       out_back( 0,  S);
    printk("right\n");      out_back( 0, -S);
    printk("fwd-left\n");   out_back( S,  S);
    printk("fwd-right\n");  out_back( S, -S);
    printk("back-left\n");  out_back(-S,  S);
    printk("back-right\n"); out_back(-S, -S);

    // tip the hopper, then bring it back
    printk("tip hopper\n");
    hopper_extend();
    k_msleep(HOPPER_TEST_TRAVEL_MS);
    hopper_stop();
    k_msleep(HOPPER_TEST_PAUSE_MS);
    hopper_retract();
    k_msleep(HOPPER_TEST_TRAVEL_MS);
    hopper_stop();
    k_msleep(HOPPER_TEST_PAUSE_MS);

    // open and close the gripper
    printk("gripper open\n");
    gripper_set(GRIP_OPEN);
    k_msleep(GRIP_MS + 600);
    printk("gripper close\n");
    gripper_set(GRIP_CLOSED);
    k_msleep(GRIP_MS + 600);

    // rotate: cw 30, ccw 60, cw 30 -> back to center
    printk("rotate cw 30\n");   spin_deg(-1, 30);
    printk("rotate ccw 60\n");  spin_deg(+1, 60);
    printk("rotate cw 30\n");   spin_deg(-1, 30);

    printk("--- demo complete ---\n");
    return 0;
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
