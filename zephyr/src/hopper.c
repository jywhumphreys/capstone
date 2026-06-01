#include "hopper.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define HOPPER_NODE DT_NODELABEL(hopper)

// long enough to reach the limit switch with margin. this short-throw unit
// fully travels in ~2s, well under its 2-min on-time duty limit
#define HOPPER_TRAVEL_MS  3000u

static const struct gpio_dt_spec ain1 = GPIO_DT_SPEC_GET(HOPPER_NODE, ain1_gpios);
static const struct gpio_dt_spec ain2 = GPIO_DT_SPEC_GET(HOPPER_NODE, ain2_gpios);

static bool     moving;
static uint32_t move_start;

int hopper_init(void)
{
    if (!gpio_is_ready_dt(&ain1) || !gpio_is_ready_dt(&ain2)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&ain1, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&ain2, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    moving = false;
    return 0;
}

void hopper_extend(void)
{
    gpio_pin_set_dt(&ain1, 1);
    gpio_pin_set_dt(&ain2, 0);
    move_start = k_uptime_get_32();
    moving = true;
}

void hopper_retract(void)
{
    gpio_pin_set_dt(&ain1, 0);
    gpio_pin_set_dt(&ain2, 1);
    move_start = k_uptime_get_32();
    moving = true;
}

void hopper_stop(void)
{
    gpio_pin_set_dt(&ain1, 0);
    gpio_pin_set_dt(&ain2, 0);
    moving = false;
}

void hopper_tick(void)
{
    if (moving && (k_uptime_get_32() - move_start) >= HOPPER_TRAVEL_MS) {
        hopper_stop();
    }
}
