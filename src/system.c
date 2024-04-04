#include "system.h"

#include <stm32f4xx.h>

//SysTick Setup/////////////////////////////////////////////////////////////////
volatile uint64_t ticks = 0;

__attribute__ ((interrupt("IRQ")))
void systick_irq_vector(void) {
    ticks++;
}

void setup_systick(void) {
    // 
    
    // SysTick consists of four registers:
    // - Control and Status register
    // - Counter reload value register (when does the systick reset?)
    // - Counter current value regster
    // - calibration value register.

    /* systick_set_frequency(SYSTICK_FREQ, CPU_FREQ); */
    /* systick_counter_enable(); */
    /* systick_interrupt_enable(); */


    // SysTick Clock = AHB/8 = SYS_CLOCK/8
    // Systick Clock = 10,000,000 Hz

    // to trigger an interrupt 100x a second, Reload Value = SysTick_CLOCK/100.
    // Reload Value = 100,000
    // enable SysTick counter and interupt (this function does these things)
    SysTick_Config(100000);

    NVIC_SetPriority(SysTick_IRQn, 0); // make this the most important
    __enable_irq(); // clear PRIMASK to enable exceptions to execute.
    __enable_fault_irq();
}

uint64_t get_ticks(void) {
    return ticks;
}




//RCC Setup/////////////////////////////////////////////////////////////////////
void setup_rcc(void) {
    // set up the clock

    // there are 4 registers that need to be configured:
    // - RCC_CR :: RCC clock control register
    // - RCC_PLLCFGR :: RCC PLL configuration register
    // - RCC_CFGR :: RCC clock configuration register
    // - RCC_CIR :: RCC clock interrupt register
    //
    // The PLL needs to be configured before it is enabled.

    // see page 105 of RM0368 for a complete description of PLL formulas.
    // VCO_CLOCK = 8MHz * (PLLN / PLLM)
    // SYS_CLOCK = VCO_CLOCK / PLLP
    // USB_CLOCK = VCO_CLOCK / PLLQ
    //
    // USB_CLOCK: 48Mhz.
    // SYS_CLOCK: 80 MHz
    //
    // To find PLLN and PLLM, we need to find values for PLLP and PLLQ that
    // satisfy the above equations.
    //
    // Here is a possible solution (values from the ST configuration tool):
    //   - PLLN = 80
    //   - PLLM = 8
    //   - PLLQ = 7
    //   - PLLP = 2
    RCC->PLLCFGR =
        // PLL multipliers/divisors
        (80 << RCC_PLLCFGR_PLLN_Pos) |
        (8  << RCC_PLLCFGR_PLLM_Pos) |
        (7  << RCC_PLLCFGR_PLLQ_Pos) |
        (2  << RCC_PLLCFGR_PLLP_Pos) |

        // PLL source
        RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CFGR =
        // APB[1,2] prescaler: divide by 1
        
        // AHB prescaler: divide by 1
        RCC_CFGR_HPRE_DIV1 |
        // use PLL as the system clock
        RCC_CFGR_SW_PLL;
        
    RCC->CR =
        RCC_CR_PLLON |     // enable PLL
        RCC_CR_HSEON;      // enable 8MHz external clock from ST-link

    // disable all interrupts triggered on clock status events.
    RCC->CIR = 0;
}

//USB Setup/////////////////////////////////////////////////////////////////////
void setup_usb(void) {
    // Core Initialization
    
}

//GPIO Setup////////////////////////////////////////////////////////////////////
void setup_gpio(void) {
    /* rcc_periph_clock_enable(RCC_GPIOA); */
    /* rcc_periph_clock_enable(RCC_GPIOC); */

    /* // set up PWM on LED */
    /* gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5); */
    /* gpio_set_af(GPIOA, GPIO_AF1, GPIO5); */

    /* // input pin */
    /* gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO13); */

    // Enable GPIO Peripheral clock
    // PORTA, Status LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set alternate functions to 0 for GPIO
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL0;

    // Configure GPIOs for input/output
#define GPIO_MODER_INPUT_MODE  (0x0)
#define GPIO_MODER_OUTPUT_MODE (0x1)
#define GPIO_MODER_AF_MODE     (0x2)
#define GPIO_MODER_ANALOG_MODE (0x3)
    
    GPIOA->MODER |= GPIO_MODER_OUTPUT_MODE << GPIO_MODER_MODER5_Pos;

    // use push/pull mode for driving LEDs (default state)
    // GPIOA->OTYPER = 0;

    // Low speed is probably fine
    // GPIOA->OSPEEDR = 0;

    // no pullup or pulldown needed for LEDs
    // GPIOA->PUPDR = 0;
}

void setup_system(void) {
    setup_rcc();

    setup_systick();
    setup_usb();
    setup_gpio();
}


//Exception Handlers////////////////////////////////////////////////////////////
void void_vector(void) {
    while (1); // never return
}

// main function symbol
extern int main(void);

// data section addresses defined in linker
extern uint8_t _data_section_vma_start;
extern uint8_t _data_section_vma_end;
extern uint8_t _data_section_lma_start;

// bss section addressed defined in linker
extern uint8_t _bss_section_vma_start;
extern uint8_t _bss_section_vma_end;

void reset_handler(void) {
    // copy .data section from ROM into RAM
    uint8_t* src = &_data_section_lma_start;
    uint8_t* dst = &_data_section_vma_start;

    while (dst < &_data_section_vma_end) {
        *dst++ = *src++;
    }

    // zero out .bss section
    for (uint8_t* pos = &_bss_section_vma_start;
         pos < &_bss_section_vma_end;
         pos++) {
        
        *pos = 0;
    }

    // jump to main function
    main();

    // never return
}


// filled by the linker script
extern uint32_t _stack;

//Vector Table//////////////////////////////////////////////////////////////////
__attribute__((section(".vectors")))
const vector_table_t vector_table = {
    .stack_start         = &_stack,
    .reset_vector        = &reset_handler,
    .nmi_vector          = &void_vector,
    .hardfault_vector    = &void_vector,
    .memmanage_vector    = &void_vector,
    .busfault_vector     = &void_vector,
    .usagefault_vector   = &void_vector,
    .reserved_vector_7   = &void_vector,
    .reserved_vector_8   = &void_vector,
    .reserved_vector_9   = &void_vector,
    .reserved_vector_10  = &void_vector,
    .svcall_vector       = &void_vector,
    .debugmonitor_vector = &void_vector,
    .reserved_vector_13  = &void_vector,
    .pendsv_vector       = &void_vector,
    .systick_vector      = &systick_irq_vector,

    .external_interrupt_vector = { &void_vector },
};
