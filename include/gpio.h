#ifndef GPIO_H
#define GPIO_H

#include "stm32f401xe.h"
typedef enum {
    GPIO_PIN0 = 1 << 0,
    GPIO_PIN1 = 1 << 1,
    GPIO_PIN2 = 1 << 2,
    GPIO_PIN3 = 1 << 3,
    GPIO_PIN4 = 1 << 4,
    GPIO_PIN5 = 1 << 5,
    GPIO_PIN6 = 1 << 6,
    GPIO_PIN7 = 1 << 7,
    GPIO_PIN8 = 1 << 8,
    GPIO_PIN9 = 1 << 9,
    GPIO_PIN10 = 1 << 10,
    GPIO_PIN11 = 1 << 11,
    GPIO_PIN12 = 1 << 12,
    GPIO_PIN13 = 1 << 13,
    GPIO_PIN14 = 1 << 14,
    GPIO_PIN15 = 1 << 15,
} GPIO_PIN_Type;

void gpio_pin_set(GPIO_TypeDef* port, const GPIO_PIN_Type pin);
void gpio_pin_clr(GPIO_TypeDef* port, const GPIO_PIN_Type pin);
void gpio_pin_tgl(GPIO_TypeDef* port, const GPIO_PIN_Type pin);

void gpio_pin_read(const GPIO_TypeDef* port, const GPIO_PIN_Type pin);

#endif
