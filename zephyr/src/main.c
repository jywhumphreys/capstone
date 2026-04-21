#include <zephyr/kernel.h>

int main(void)
{
    while (1) {
        k_msleep(1000);
    }
}