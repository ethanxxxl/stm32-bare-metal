#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

#define CPU_FREQ     (84000000)
#define SYSTICK_FREQ (1000)

//Vector Table////////////////////////////////////////////////////////////////
typedef void(*irq_vector)(void);

#define NUM_INTERRUPTS 52
typedef struct {
    uint32_t*  stack_start;
    irq_vector reset_vector;
    irq_vector nmi_vector;
    irq_vector hardfault_vector;
    irq_vector memmanage_vector;
    irq_vector busfault_vector;
    irq_vector usagefault_vector;
    irq_vector reserved_vector_7;
    irq_vector reserved_vector_8;
    irq_vector reserved_vector_9;
    irq_vector reserved_vector_10;
    irq_vector svcall_vector;
    irq_vector debugmonitor_vector;
    irq_vector reserved_vector_13;
    irq_vector pendsv_vector;
    irq_vector systick_vector;
    irq_vector external_interrupt_vector[NUM_INTERRUPTS];
} vector_table_t;

void void_vector(void);


/* SYSTICK SETUP
 */
void setup_systick(void);
uint64_t get_ticks(void);

/* RCC SETUP
 */
void setup_rcc(void);

/* USB Device Mode Setup
 */
void setup_usb(void);

/* GPIO SETUP
 */
void setup_gpio(void);

/* SETUP EVERYTHING
 */
void setup_system(void);

#endif
