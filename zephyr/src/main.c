#include <zephyr/kernel.h>
#include "comms.h"

int main(void)
{
    uart_init();

    while (1) {
        uart_tick();
        k_msleep(1);
    }
}