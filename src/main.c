#include "system.h"
#include "gpio.h"

#include <stdbool.h>
#include <stm32f4xx.h>

int main(void) {
    setup_system();

    uint64_t last_time = get_ticks();

    // either systick isn't getting started, or the interupt vector isn't
    // getting called.

    gpio_pin_set(GPIOA, GPIO_PIN5);

    while(true) {
        if (get_ticks() >= last_time + 20) {
            gpio_pin_tgl(GPIOA, GPIO_PIN5);
            last_time = get_ticks();
        }
    }

    return 0;
}
