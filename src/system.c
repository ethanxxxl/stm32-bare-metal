#include "system.h"
#include "stm32f401xe.h"

#include <stm32f4xx.h>
#include <stdbool.h>

#include "gpio.h"

#define FIELD_VAL(REG, MASK, SHIFT) (((REG) & (MASK)) >> SHIFT)

#define STATUS_LED0 (GPIO_PIN0)
#define STATUS_LED1 (GPIO_PIN1)
#define STATUS_LED2 (GPIO_PIN4)
#define STATUS_LED3 (GPIO_PIN8)

//////////////////////////////// SysTick Setup /////////////////////////////////
volatile uint64_t ticks = 0;

__attribute__ ((interrupt("IRQ")))
void systick_irq_vector(void) {
    ticks++;
}

void setup_systick(void) {
    // SysTick consists of four registers:
    // - Control and Status register
    // - Counter reload value register (when does the systick reset?)
    // - Counter current value regster
    // - calibration value register.

    /* systick_set_frequency(SYSTICK_FREQ, CPU_FREQ); */
    /* systick_counter_enable(); */
    /* systick_interrupt_enable(); */

    // SysTick Clock = AHB/8 = SYS_CLOCK/8
    // Systick Clock = 84 MHz

    // to trigger an interrupt 100x a second, Reload Value = SysTick_CLOCK/100.
    // Reload Value = 100,000
    // enable SysTick counter and interupt (this function does these things)
    SysTick_Config(840000);

    NVIC_SetPriority(SysTick_IRQn, 0); // make this the most important
    NVIC_SetPriorityGrouping(0x3); // NVIC_PRIORITYGROUP_4
    __enable_irq(); // clear PRIMASK to enable exceptions to execute.
    __enable_fault_irq();
}

uint64_t get_ticks(void) {
    return ticks;
}

/* This is some HAL code I stole for error checking in the RCC secion. */
#define RCC_FLAG_MASK ((uint8_t)0x1FU)
#define __HAL_RCC_GET_FLAG(__FLAG__)                                           \
  (((((((__FLAG__) >> 5U) == 1U)                                               \
          ? RCC->CR                                                            \
          : ((((__FLAG__) >> 5U) == 2U)                                        \
                 ? RCC->BDCR                                                   \
                 : ((((__FLAG__) >> 5U) == 3U) ? RCC->CSR : RCC->CIR))) &      \
     (1U << ((__FLAG__)&RCC_FLAG_MASK))) != 0U)                                \
       ? 1U                                                                    \
       : 0U)

#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3B)

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_BORRST                  ((uint8_t)0x79)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

////////////////////////////////// RCC Setup ///////////////////////////////////

void setup_rcc(void) {
    SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
    SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_CR_VOS_1);

    /*************************** HSE Configuration ****************************/
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    // wait for HSE to be ready
    //while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0);
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0);

    /*************************** PLL Configuration ****************************/
    // disable Main PLL so it can be modified
    MODIFY_REG(RCC->CR, RCC_CR_PLLON, 0);
    MODIFY_REG(RCC->CR, RCC_CR_PLLI2SON, 0);

    // wait for PLLs to shutdown
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != 0);

    WRITE_REG(RCC->PLLCFGR,
              RCC_PLLCFGR_PLLSRC_HSE        |
              (4   << RCC_PLLCFGR_PLLM_Pos) |
              (168 << RCC_PLLCFGR_PLLN_Pos) |
              (0x1 << RCC_PLLCFGR_PLLP_Pos) | // divide by 4 (pg. 106)
              (7   << RCC_PLLCFGR_PLLQ_Pos));
    
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    // wait for PLL to be ready
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 0);

    /************************* Increase Flash Latency *************************/
    // in order to correctly read flash memory at the frequency chosen, the
    // number of wait states needs tobe programmed in the flash access control
    // register.  pg 46
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS);

    if ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_2WS)
        while(true); // error, loop forever.

    /*************************** HCLK Configuration ***************************/
    // set APB clocks to max division so that they don't transition through a
    // non-spec frequency when configuring AHB.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);

    // set AHB prescaler
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    /************************* SYSCLCK Configuration **************************/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /************************** PCLK1 Configuration ***************************/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);

    /************************** PCLK2 Configuration ***************************/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
          
    // disable all interrupts triggered on clock status events.
    RCC->CIR = 0;
}

////////////////////////////////// USB Setup ///////////////////////////////////
USB_OTG_DeviceTypeDef *USB_OTG_FS_DEVICE =
    (USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);

USB_OTG_INEndpointTypeDef* USB_OTG_INEndpoints =
    (USB_OTG_INEndpointTypeDef*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE);

USB_OTG_OUTEndpointTypeDef* USB_OTG_OUTEndpoints =
    (USB_OTG_OUTEndpointTypeDef*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE);

void setup_usb(void) {
    // enable USB ORG FS clock
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN);
    
    /*
      Core Initialization (pg. 764)

      1. Program the following fields in the OTG_FS_GAHBCFG register:

         - Global interrupt mask bit GINTMSK = 1
         - RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
         - Periodic TxFIFO empty level

      2. Program the following fields in the OTG_FS_GUSBCFG register:

         - Host Navigation Protocol (HNP) capable bit
         - Session Request Protocol (SRP) capable bit
         - FS timeout calibration field
         - USB turnaround time field

      3. The software must unmask the following bits in the OTG_FS_GINTMSK
         register:

         - OTG interrupt mask
         - Mode mismatch interrupt mask

      4. The software can read the CMOD bit in OTG_FS_GINTSTS to determine
         whether the OTG_FS controller is operating in host or device mode
     */

    USB_OTG_FS->GOTGCTL =
        USB_OTG_GOTGCTL_CIDSTS |
        USB_OTG_GOTGCTL_BSVLD  ;
    
    // RxFIFO not in this register???
    USB_OTG_FS->GAHBCFG =
        USB_OTG_GAHBCFG_GINT
        | USB_OTG_GAHBCFG_TXFELVL; // TXFE interrupt indicates when TxFIFO is completely empty

    // HNP and SRP not necessary for this application
    USB_OTG_FS->GUSBCFG =
        USB_OTG_GUSBCFG_FDMOD              | // force device mode
        (0x0 << USB_OTG_GUSBCFG_TOCAL_Pos) | // this is just a guess
        (0xd << USB_OTG_GUSBCFG_TRDT_Pos)  ; // no IN token response time delay


    // clear interrupt flags before enabling the interrupts
    USB_OTG_FS->GINTSTS =
        USB_OTG_GINTSTS_OTGINT |
        USB_OTG_GINTSTS_USBRST |
        USB_OTG_GINTSTS_SRQINT |
        USB_OTG_GINTSTS_MMIS   ;

    // enable the interrupts
    USB_OTG_FS->GINTMSK =
        USB_OTG_GINTMSK_OTGINT | // OTG interrupt mask
        USB_OTG_GINTMSK_USBRST |
        USB_OTG_GINTMSK_SRQIM  |
        USB_OTG_GINTMSK_MMISM  ; // mode mismatch interrupt mask  

    // enable global NVIC Interrupt
    __NVIC_EnableIRQ(OTG_FS_IRQn);

    /*
      Device Initialization

      In OTG_FS_DCFG register:

        - program device speed
        - Non-zero-length status OUT handshake
      
      Program the OTG_FS_GINTMSK register to unmask the following interrupts:

        - USB reset
        - Enumeration done
        - Early suspend
        - USB suspend
        - SOF

      Program the VBUSBSEN bit in the OTG_FS_GCCFG register to enable Vbus
      sensing in "B" device mode and supply the 5 volts across the pullup
      resistor on the DP line.

      Wait for the USBRST interrupt in OTG_FS_GINTSTS. This interrupt indicates
      the end of reset on the USB. On receiving this interrupt, the application
      must read the OTG_FS_DSTS register to determine the enumeration speed and
      perform the steps listed in Endpoint initialization on enumeration
      completion.

      At this point, the device is ready to accept SOF packets and perfom
      control transfers on control endpoint 0.
    
     */

    /* Setup Device Configuration Register */
    USB_OTG_FS_DEVICE->DCFG |= USB_OTG_DCFG_NZLSOHSK;
    USB_OTG_FS_DEVICE->DCFG |= USB_OTG_DCFG_DSPD; // 0x3 is the only valid option

    /* Enable Interrupts */
    USB_OTG_FS->GINTSTS |= // clear interrupts before enabling them
        USB_OTG_GINTSTS_USBRST
        | USB_OTG_GINTSTS_ENUMDNE_Msk   
        | USB_OTG_GINTSTS_ESUSP         
        | USB_OTG_GINTSTS_USBSUSP       
        | USB_OTG_GINTSTS_SOF;

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST;
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ENUMDNEM;
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM;
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ESUSPM;
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

    /* Enable Vbus Sensing in B device mode */
    USB_OTG_FS->GCCFG |=
        USB_OTG_GCCFG_VBUSBSEN |
        USB_OTG_GCCFG_PWRDWN   ;
        //USB_OTG_GCCFG_NOVBUSSENS;

    //usb_reset_init();
    //usb_enumeration_complete_init();
    gpio_pin_clr(GPIOA, STATUS_LED0);
}

void usb_reset_init(void) {
    /*
      Endpoint Initialization/Completion

      1. Set the NAK bit for all OUT endpoints

        - SNAK = 1 in OTG_FS_DOEPCTLx (for all OUT endpoints)

      2. Unmask the following interrupt bits

         - INEP0 = 1 in OTF_FS_DAINTMSK (control 0 IN endpoint)
         - OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
         - STUP = 1 in DOEPMSK
         - XFRC = 1 in DOEPMSK
         - XFRC = 1 in DIEPMSK
         - TOC = 1 in DIEPMSK

      3. Set up the Data FIFO RAM for each of the FIFOs

         - Program the OTG_FS_GRXFSIZ register, to be able to receive control
           OUT data and setup data.  If thresholding is not enabled, at a
           minimum, this must be equal to 1 max packet size of control endpoint
           0 + 2 words ( for the status of the control OUT data packet) + 10
           words (for setup packets)
        
        - Program the OTG_FS_TX0FSIZ register (depending on the FIFO number
          chosen) to be able to transmit control IN data.  At a minimum, this
          must be equal to 1 max packet size of control endpoint 0.

      4. Program the following fields in the endpoint-specific registers for
         control OUT endpoint 0 to receive a SETUP packet

         - STUPCNT = 3 in OTG_FS_DOEPTSIZ0 (to receive up to 3 back-to-back
           SETUP packets)

      At this point, all initialization required to receive SETUP packets is
      done
    */

    // Set NAK bit for all OUT endpoints
    USB_OTG_OUTEndpoints[0].DOEPCTL |= USB_OTG_DOEPCTL_SNAK;

    // Unmask interrupt bits (clear them first)
    // using equal because for some reason the reset value is getting screwed up?
     USB_OTG_FS_DEVICE->DAINTMSK =
        0x1 << USB_OTG_DAINTMSK_IEPM_Pos | // Control 0 IN endpoint
        0x1 << USB_OTG_DAINTMSK_OEPM_Pos ; // Contorl 0 OUT endpoint

    USB_OTG_FS_DEVICE->DOEPMSK |=
        USB_OTG_DOEPMSK_STUPM |
        USB_OTG_DOEPMSK_XFRCM ;

    USB_OTG_FS_DEVICE->DIEPMSK |=
        USB_OTG_DIEPMSK_XFRCM |
        USB_OTG_DIEPMSK_TOM   ;

    // Setup Data FIFO RAM for each FIFO

    // OUT endpoint: RX FIFO
    USB_OTG_FS->GRXFSIZ = 256; // maximum size of the buffer allowed

    // IN endpoint: TX FIFO
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ |=
        (256 << USB_OTG_TX0FD_Pos); // XXX there is also a ram start adress. I
                                    // assume that it refers to the FIFO ram
                                    // location in the USB peripheral.  The
                                    // initialization instructions do not
                                    // specify that this needs to be
                                    // initialized.

    USB_OTG_INEndpoints[0].DIEPTSIZ |= 0x7F; // maximum size allowed.

    // NOTE: endpoint0 has a different format for this register!
    USB_OTG_OUTEndpoints[0].DOEPTSIZ =
        (0x3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
        (0x1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos)  |
        (0x7F << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) ;
}

void usb_enumeration_complete_init(void) {
    /*
      Endpoint Initializtion on Enumeration Completion

      1. On the Enumeration Done Interrupt (ENUMDNE in OTG_FS_GINTSTS), read the
      OTG_FS_DSTS register to determine the enumeration speed.

      2. Program the MPSIZ field in OTG_FS_DIEPCTL0 to set the maximum packet
      size. This step configures control endpoint 0.  The maximum packet size
      for a control endpoint depends on the enumeration speed.

      At this point, the device is ready to receive SOF packets and is
      configured to perform control transfers on control endpoint 0.
    */

    uint32_t enumeration_speed =
        (USB_OTG_FS_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) >> USB_OTG_DSTS_ENUMSPD_Pos;

    USB_OTG_INEndpoints[0].DIEPCTL &=
        (USB_OTG_INEndpoints[0].DIEPCTL & (~USB_OTG_DIEPCTL_MPSIZ_Msk)) |
        (enumeration_speed << USB_OTG_DIEPCTL_MPSIZ_Pos);
}

void usb_endpoint_activation(void) {

}
    
/** returns true if the interrupt bit is set, otherwise, false. */
bool usb_gotg_status(uint32_t interrupt_mask) {
    return (USB_OTG_FS->GOTGINT & interrupt_mask) != 0;
}

enum usb_gintsts {
    CMOD               = 0,
    MMIS               = 1,
    OTGINT             = 2,
    SOF                = 3,
    RXFLVL             = 4,
    NPTXFE             = 5,
    GINAKEFF           = 6,
    GONAKEFF           = 7,
    
    ESUSP              = 10,
    USBSUSP            = 11,
    USBRST             = 12,
    ENUMDNE            = 13,
    ISOODRP            = 14,
    EOPF               = 15,

    IEPINT             = 18,
    OEPINT             = 19,
    IISOIXFR           = 20,
    IPXFR_INCOMPISOOUT = 21,

    HPRTINT            = 24,
    HCINT              = 25,
    PTXFE              = 26,

    CIDSCHG            = 28,
    DISCINT            = 29,
    SRQINT             = 30,
    WKUINT             = 31,
};

/** returns true if the interrupt bit is set, otherwise, false. */
bool usb_gintsts_get_interrupt(enum usb_gintsts interrupt) {
    return (USB_OTG_FS->GINTSTS & (1 << interrupt)) != 0;
}

void usb_gintsts_set_interrupt(enum usb_gintsts interrupt) {
    SET_BIT(USB_OTG_FS->GINTSTS, (1 << interrupt));
}

void usb_global_interrupt_handler(void) {
    if (usb_gotg_status(USB_OTG_GOTGINT_SEDET)) {
        gpio_pin_clr(GPIOA, STATUS_LED0);
        SET_BIT(USB_OTG_FS->GOTGINT, USB_OTG_GOTGINT_SEDET);
    }
}

#define USB_REQUEST_TYPE_STANDARD 0
#define USB_REQUEST_TYPE_CLASS 1
#define USB_REQUEST_TYPE_VENDOR 2
#define USB_REQUEST_RECIPIENT_DEVICE 0
#define USB_REQUEST_RECIPIENT_INTERFACE 1
#define USB_REQUEST_RECIPIENT_ENDPOINT 2
#define USB_REQUEST_RECIPIENT_OTHER 3
#define USB_REQUEST_DIRECTION_HOST_TO_DEVICE 0
#define USB_REQUEST_RECIPIENT_DEVICE_TO_HOST 1

struct usb_request_type {
    uint8_t direction: 1;
    uint8_t type: 2;
    uint8_t recipient: 5;
};

enum USB_REQUEST_CODES {
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    SET_FEATURE = 3,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12,
};

enum USB_DESCRIPTOR_TYPES {
    DEVICE = 1,
    CONFIGURATION = 2,
    STRING = 3,
    INTERFACE = 4,
    ENDPOINT = 5,
    DEVICE_QUALIFIER = 6,
    OTHER_SPEED_CONFIGURATION = 7,
    INTERFACE_POWER = 8,
};

void usb_handle_setup_packet(uint8_t data[8]) {
    struct usb_request_type request_type = *(struct usb_request_type*)&data[0];
    enum USB_REQUEST_CODES request = data[1];
    uint16_t value = *(uint16_t*)&data[2];
    uint16_t index = *(uint16_t*)&data[4];
    uint16_t length = *(uint16_t*)&data[6];

    (void)request_type;
    (void)index;
    (void)length;

    /************************ Standard Device Requests ************************/
    switch (request) {
    case GET_STATUS:
        
        break;
    case CLEAR_FEATURE:
        break;
    case SET_FEATURE:
        break;
    case SET_ADDRESS:
        MODIFY_REG(USB_OTG_FS_DEVICE->DCFG, USB_OTG_DCFG_DAD, value);
        gpio_pin_set(GPIOA, STATUS_LED2);
                
        break;
    case GET_DESCRIPTOR:
        break;
    case SET_DESCRIPTOR:
        break;
    case GET_CONFIGURATION:
        break;
    case SET_CONFIGURATION:
        break;
    case GET_INTERFACE:
        break;
    case SET_INTERFACE:
        break;
    case SYNCH_FRAME:
        break;
    }
}

void usb_read_rx_data_fifo(uint8_t *buffer, uint32_t bytes) {
    uint32_t words = bytes >> 2;
    for (uint32_t i = 0; i < words; i++) {
        // reading from any address in the RxFIFO range will pop the next word
        *(uint32_t*)(buffer + (i*4)) =
            *(volatile uint32_t*)(USB_OTG_FS_PERIPH_BASE +
                                  USB_OTG_FIFO_BASE);
    }

    // read remaining bytes from RxFIFO
    uint32_t remainder = bytes & 4;
    if (remainder == 0)
        return;

    uint32_t last_word = *(volatile uint32_t*)(USB_OTG_FS_PERIPH_BASE +
                                               USB_OTG_FIFO_BASE);
    for (uint32_t i = 0; i < remainder; i++) {
        buffer[(bytes >> 2) + i] = ((uint8_t*)&last_word)[i];
    }

}

uint32_t last_three_setup_packets[2 * 3];

void usb_device_out_endpoint_interrupt_handler(void) {
    // OUT endpoint zero
    if ((USB_OTG_FS_DEVICE->DAINT & (0x0 << USB_OTG_DAINT_OEPINT_Pos)) == 0) {
        volatile uint32_t* DOEPINT = &USB_OTG_OUTEndpoints[0].DOEPINT;

        /************************ Handle Setup Packet *************************/
        if ((*DOEPINT & USB_OTG_DOEPINT_STUP_Msk) != 0) {
            gpio_pin_set(GPIOA, STATUS_LED3);

            usb_handle_setup_packet(buffer);

            // TODO: are you supposed to do something here?
            *DOEPINT |= USB_OTG_DOEPINT_STUP;
        }

        // Transfer Complete
        if ((*DOEPINT & USB_OTG_DOEPINT_XFRC) != 0) {
            *DOEPINT |= USB_OTG_DOEPINT_XFRC;
        }
    }
}

void usb_device_in_endpoint_interrupt_handler(void) {
    // IN endpoint zero
    if ((USB_OTG_FS_DEVICE->DAINT & (0x0 << USB_OTG_DAINT_IEPINT_Pos)) == 0) {
        // Transfer Complete
        volatile uint32_t* DIEPINT = &USB_OTG_INEndpoints[0].DIEPINT;
        if ((*DIEPINT & USB_OTG_DIEPINT_XFRC) != 0) {
            // TODO: are you supposed to do something here?
            *DIEPINT |= USB_OTG_DIEPINT_XFRC;
        }

        // Timeout Condition
        if ((*DIEPINT & USB_OTG_DIEPINT_TOC_Msk) != 0) {
            *DIEPINT |= USB_OTG_DIEPINT_TOC;
        }        
    }
}

void usb_interrupt_handler(void) {
    // Session Request (plugged in USB)
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SRQINT_Msk) != 0) {
        // not much to do here, just signal that something happened.
        gpio_pin_set(GPIOA, STATUS_LED0);

        // switch to idle state
        MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD11, 0x02); // DM: PULLDOWN
        MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD12, 0x01); // DP: PULLUP

        SET_BIT(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_SRQINT); // clear interrupt
        
    }

    /********************** USB_OTG_FS Global Interrupt ***********************/
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OTGINT_Msk) != 0 ) {
        usb_global_interrupt_handler();
    }

    if (usb_gintsts_get_interrupt(RXFLVL)) {
        uint32_t receive_status = USB_OTG_FS->GRXSTSP;

        uint32_t byte_count = FIELD_VAL(receive_status,
                                        USB_OTG_GRXSTSP_BCNT,
                                        USB_OTG_GRXSTSP_BCNT_Pos);

        uint32_t packet_status = FIELD_VAL(receive_status,
                                          USB_OTG_GRXSTSP_PKTSTS,
                                          USB_OTG_GRXSTSP_PKTSTS_Pos);

        uint8_t buffer[byte_count];

        usb_read_rx_data_fifo(buffer, byte_count);

        switch (packet_status) {
        case 1: // GLOBAL OUT NAK
            break;
        case 2: // OUT data packet received
            break;
        case 3: // OUT transfer completed (triggers an interrupt)
            break;
        case 4: // SETUP transaction completed (triggers an interrupt)
            // moving into a data stage...
            break;
        case 6: { // SETUP data packet received
            uint32_t recvd_packets = FIELD_VAL(USB_OTG_OUTEndpoints[0].DOEPTSIZ,
                                               USB_OTG_DOEPTSIZ_STUPCNT,
                                               USB_OTG_DOEPTSIZ_STUPCNT_Pos);
                
            usb_read_rx_data_fifo(last_three_setup_packets + (8 * recvd_packets),
                                  8)
                break;                
            }

        }
        
        
        SET_BIT(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_RXFLVL);
    }
    
    // OUT endpoint interrupts
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT_Msk) != 0) {
        usb_device_out_endpoint_interrupt_handler();
    }

    // IN endpoint interrupts
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT_Msk) != 0) {
        usb_device_in_endpoint_interrupt_handler();
    }

    // USB Reset
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST_Msk) != 0) {
        usb_reset_init();
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST; // clear interupt
    }
    
    // Enumeration done
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE_Msk) != 0) {
        usb_enumeration_complete_init();
        SET_BIT(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_ENUMDNE); // clear interrupt
    }

    // Early suspend
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ESUSP_Msk) != 0) {
        
    }

    // USB suspend
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBSUSP_Msk) != 0) {
        
    }

    // SOF
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF_Msk) != 0) {
        gpio_pin_set(GPIOA, STATUS_LED1);

        // right now, nothing needs to be done with start of frame packets.
        // They only contain the frame number. Since this application doesn't do
        // anythint with the frame number, it can be ignored here.

        SET_BIT(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_SOF); // clear interrupt
    }
}

////////////////////////////////// GPIO Setup //////////////////////////////////
void setup_gpio(void) {
    /************************ Enable Peripheral Clocks ************************/
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);

    // Configure GPIOs for input/output
#define GPIO_MODER_INPUT_MODE  (0x0)
#define GPIO_MODER_OUTPUT_MODE (0x1)
#define GPIO_MODER_AF_MODE     (0x2)
#define GPIO_MODER_ANALOG_MODE (0x3)

    /*************************** Set Mode Registers ***************************/
    MODIFY_REG(GPIOA->MODER,
               /******************* GPIOA MODER CLEAR MASK ********************/
               // UART
               GPIO_MODER_MODE2  |
               GPIO_MODER_MODE3  |
               
               // Buit-In LED
               GPIO_MODER_MODE5  |

               // Breadboard Status LEDs
               GPIO_MODER_MODE0  |
               GPIO_MODER_MODE1  |
               GPIO_MODER_MODE4  |
               GPIO_MODER_MODE8  |

               // USB
               GPIO_MODER_MODE9  |
               GPIO_MODER_MODE11 |
               GPIO_MODER_MODE12,

               /******************** GPIOA MODER SET MASK *********************/
               // UART
               (GPIO_MODER_AF_MODE << GPIO_MODER_MODE2_Pos)      |
               (GPIO_MODER_AF_MODE << GPIO_MODER_MODE3_Pos)      |

               // Built-In LED
               (GPIO_MODER_OUTPUT_MODE << GPIO_MODER_MODE5_Pos)  |

               // Breadboard Status LEDs
               (GPIO_MODER_OUTPUT_MODE << GPIO_MODER_MODE0_Pos)  |
               (GPIO_MODER_OUTPUT_MODE << GPIO_MODER_MODE1_Pos)  |
               (GPIO_MODER_OUTPUT_MODE << GPIO_MODER_MODE4_Pos)  |
               (GPIO_MODER_OUTPUT_MODE << GPIO_MODER_MODE8_Pos)  |

               // USB
               (GPIO_MODER_AF_MODE << GPIO_MODER_MODE9_Pos)      | // USB_OTG VBUS
               (GPIO_MODER_AF_MODE << GPIO_MODER_MODE11_Pos)     | // USB_OTG DM
               (GPIO_MODER_AF_MODE << GPIO_MODER_MODE12_Pos));     // USB_OTG DP

    /************************ Set Alternate Functions *************************/
    // GPIOA Low
    MODIFY_REG(GPIOA->AFR[0],
               GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3,
               (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos));

    // GPIOA High
    MODIFY_REG(GPIOA->AFR[1],
               GPIO_AFRH_AFSEL9 |
               GPIO_AFRH_AFSEL11 |
               GPIO_AFRH_AFSEL12,
               
               (10 << GPIO_AFRH_AFSEL9_Pos)  |  // USB_OTG VBUS
               (10 << GPIO_AFRH_AFSEL11_Pos) |  // USB_OTG DM
               (10 << GPIO_AFRH_AFSEL12_Pos));  // USB_OTG DP

    /******************************* Push/Pull ********************************/

    // use push/pull mode for driving LEDs (default state)
    GPIOA->OTYPER |= 0;

    /********************************* Speed **********************************/
    // Low speed is probably fine
    GPIOA->OSPEEDR |=
        (0x3 << 18) |
        (0x3 << 22) |
        (0x3 << 24) ;

    /********************** Set Pullups/Pulldown/Neither **********************/

    // no pullup or pulldown needed for LEDs
    GPIOA->PUPDR |=
        (0x2 << GPIO_PUPDR_PUPD0_Pos)  |
        (0x2 << GPIO_PUPDR_PUPD1_Pos)  |
        (0x2 << GPIO_PUPDR_PUPD4_Pos)  |
        (0x2 << GPIO_PUPDR_PUPD8_Pos)  |
        
        (0x0 << GPIO_PUPDR_PUPD9_Pos)  |  // USB_OTG VBUS
        (0x2 << GPIO_PUPDR_PUPD11_Pos) |  // USB_OTG DM
        (0x2 << GPIO_PUPDR_PUPD12_Pos) ;  // USB_OTG DP
}

void setup_system(void) {
    setup_rcc();

    setup_systick();
    setup_gpio();
    setup_usb();
}


////////////////////////////// Exception Handlers //////////////////////////////
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

///////////////////////////////// Vector Table /////////////////////////////////

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

    .external_interrupt_vector = {
        &void_vector,
        [67] = usb_interrupt_handler
    },
};
