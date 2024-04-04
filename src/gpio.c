#include "gpio.h"


void gpio_pin_set(GPIO_TypeDef* port, const GPIO_PIN_Type pin) {
    port->BSRR = pin;
    return;
}

void gpio_pin_clr(GPIO_TypeDef* port, const GPIO_PIN_Type pin) {
    port->BSRR = pin << 16;
    return;
}

void gpio_pin_tgl(GPIO_TypeDef* port, const GPIO_PIN_Type pin) {
    if ((port->IDR & pin) == 0)
        gpio_pin_set(port, pin);
    else
        gpio_pin_clr(port, pin);
}


void gpio_pin_read(const GPIO_TypeDef* port, const GPIO_PIN_Type pin) {
    (void)(port);
    (void)(pin);
}
