#include "system.h"
#include "gpio.h"

#include <stdbool.h>
#include <stm32f4xx.h>

int main(void) {
    setup_system();

    uint64_t last_time = get_ticks();
    

    uint32_t pins[] = {GPIO_PIN0, GPIO_PIN1, GPIO_PIN4, GPIO_PIN8};
    
    for (int i = 0; i < 4; i++) {
        gpio_pin_set(GPIOA, pins[i]);

        uint64_t t = get_ticks();
        while ( get_ticks() < t + 25);

        gpio_pin_clr(GPIOA, pins[i]);
    }

    while(true) {
        if (get_ticks() >= last_time + 33) {
            gpio_pin_tgl(GPIOA, GPIO_PIN5);
            last_time = get_ticks();
        }
    }

    return 0;
}
