/**
 * NUC472 startup ARM GCC
 * Purpose: startup file for Cortex-M4 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.0
 * Date: 22 Feb 2016
 *
 * Copyright (c) 2016 Nuvoton Technology Corp. All rights reserved.
 */
    .syntax unified
    .arch armv7-m

    .section .vector_table,"a",%progbits
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                  /* Top of Stack */
    .long   Reset_Handler               /* Reset Handler */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   MemManage_Handler           /* MPU Fault Handler            */
    .long   BusFault_Handler            /* Bus Fault Handler            */
    .long   UsageFault_Handler          /* Usage Fault Handler          */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   DebugMon_Handler            /* Debug Monitor Handler        */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    /* External Interrupts */
    .long   BOD_IRQHandler              /* 0: Brown Out detection       */
    .long   IRC_IRQHandler              /* 1: Internal RC               */
    .long   PWRWU_IRQHandler            /* 2: Power Down Wake Up        */
    .long   SRAMF_IRQHandler            /* 3: Reserved                  */
    .long   CLKF_IRQHandler             /* 4: CLKF                      */
    .long   Default_Handler             /* 5: Reserved.                 */
    .long   RTC_IRQHandler              /* 6: Real Time Clock           */
    .long   TAMPER_IRQHandler           /* 7: Tamper detection          */
    .long   EINT0_IRQHandler            /* 8: External Input 0          */
    .long   EINT1_IRQHandler            /* 9: External Input 1          */
    .long   EINT2_IRQHandler            /* 10: External Input 2         */
    .long   EINT3_IRQHandler            /* 11: External Input 3         */
    .long   EINT4_IRQHandler            /* 12: External Input 4         */
    .long   EINT5_IRQHandler            /* 13: External Input 5         */
    .long   EINT6_IRQHandler            /* 14: External Input 6         */
    .long   EINT7_IRQHandler            /* 15: External Input 7         */
    .long   GPA_IRQHandler              /* 16: GPIO Port A              */
    .long   GPB_IRQHandler              /* 17: GPIO Port B              */
    .long   GPC_IRQHandler              /* 18: GPIO Port C              */
    .long   GPD_IRQHandler              /* 19: GPIO Port D              */
    .long   GPE_IRQHandler              /* 20: GPIO Port E              */
    .long   GPF_IRQHandler              /* 21: GPIO Port F              */
    .long   GPG_IRQHandler              /* 22: GPIO Port G              */
    .long   GPH_IRQHandler              /* 23: GPIO Port H              */
    .long   GPI_IRQHandler              /* 24: GPIO Port I              */
    .long   Default_Handler             /* 25: Reserved.                */
    .long   Default_Handler             /* 26: Reserved.                */
    .long   Default_Handler             /* 27: Reserved.                */
    .long   Default_Handler             /* 28: Reserved.                */
    .long   Default_Handler             /* 29: Reserved.                */
    .long   Default_Handler             /* 30: Reserved.                */
    .long   Default_Handler             /* 31: Reserved.                */
    .long   TMR0_IRQHandler             /* 32: Timer 0                  */
    .long   TMR1_IRQHandler             /* 33: Timer 1                  */
    .long   TMR2_IRQHandler             /* 34: Timer 2                  */
    .long   TMR3_IRQHandler             /* 35: Timer 3                  */
    .long   Default_Handler             /* 36: Reserved.                */
    .long   Default_Handler             /* 37: Reserved.                */
    .long   Default_Handler             /* 38: Reserved.                */
    .long   Default_Handler             /* 39: Reserved.                */
    .long   PDMA_IRQHandler             /* 40: Peripheral DMA           */
    .long   Default_Handler             /* 41: Reserved.                */
    .long   ADC_IRQHandler              /* 42: ADC                      */
    .long   Default_Handler             /* 43: Reserved.                */
    .long   Default_Handler             /* 44: Reserved.                */
    .long   Default_Handler             /* 45: Reserved.                */
    .long   WDT_IRQHandler              /* 46: Watch Dog Timer          */
    .long   WWDT_IRQHandler             /* 47: Window Watch Dog Timer   */
    .long   EADC0_IRQHandler            /* 48: EDAC 0                   */
    .long   EADC1_IRQHandler            /* 49: EDAC 1                   */
    .long   EADC2_IRQHandler            /* 50: EDAC 2                   */
    .long   EADC3_IRQHandler            /* 51: EDAC 3                   */
    .long   Default_Handler             /* 52: Reserved.                */
    .long   Default_Handler             /* 53: Reserved.                */
    .long   Default_Handler             /* 54: Reserved.                */
    .long   Default_Handler             /* 55: Reserved.                */
    .long   ACMP_IRQHandler             /* 56: Analog Comparator        */
    .long   Default_Handler             /* 57: Reserved.                */
    .long   Default_Handler             /* 58: Reserved.                */
    .long   Default_Handler             /* 59: Reserved.                */
    .long   OPA0_IRQHandler             /* 60: OPA 0                    */
    .long   OPA1_IRQHandler             /* 61: OPA 1                    */
    .long   ICAP0_IRQHandler            /* 62: ICAP 0                   */
    .long   ICAP1_IRQHandler            /* 63: ICAP 1                   */
    .long   PWM0CH0_IRQHandler          /* 64: PWM0 CH0                 */
    .long   PWM0CH1_IRQHandler          /* 65: PWM0 CH1                 */
    .long   PWM0CH2_IRQHandler          /* 66: PWM0 CH2                 */
    .long   PWM0CH3_IRQHandler          /* 67: PWM0 CH3                 */
    .long   PWM0CH4_IRQHandler          /* 68: PWM0 CH4                 */
    .long   PWM0CH5_IRQHandler          /* 69: PWM0 CH5                 */
    .long   PWM0_BRK_IRQHandler         /* 70: PWM0 Break               */
    .long   QEI0_IRQHandler             /* 71: QEI 0                    */
    .long   PWM1CH0_IRQHandler          /* 72: PWM1 CH0                 */
    .long   PWM1CH1_IRQHandler          /* 73: PWM1 CH1                 */
    .long   PWM1CH2_IRQHandler          /* 74: PWM1 CH2                 */
    .long   PWM1CH3_IRQHandler          /* 75: PWM1 CH3                 */
    .long   PWM1CH4_IRQHandler          /* 76: PWM1 CH4                 */
    .long   PWM1CH5_IRQHandler          /* 77: PWM1 CH5                 */
    .long   PWM1_BRK_IRQHandler         /* 78: PWM1 Break               */
    .long   QEI1_IRQHandler             /* 79: QEI 1                    */
    .long   EPWM0_IRQHandler            /* 80: EPWM0                    */
    .long   EPWM0BRK_IRQHandler         /* 81: EPWM0 Break              */
    .long   EPWM1_IRQHandler            /* 82: EPWM1                    */
    .long   EPWM1BRK_IRQHandler         /* 83: EPWM1 Break              */
    .long   Default_Handler             /* 84: Reserved.                */
    .long   Default_Handler             /* 85: Reserved.                */
    .long   Default_Handler             /* 86: Reserved.                */
    .long   Default_Handler             /* 87: Reserved.                */
    .long   USBD_IRQHandler             /* 88: USB Device               */
    .long   USBH_IRQHandler             /* 89: USB Host                 */
    .long   USB_OTG_IRQHandler          /* 90: USB OTG                  */
    .long   Default_Handler             /* 91: Reserved.                */
    .long   EMAC_TX_IRQHandler          /* 92: Ethernet MAC TX          */
    .long   EMAC_RX_IRQHandler          /* 93: Ethernet MAC RX          */
    .long   Default_Handler             /* 94: Reserved.                */
    .long   Default_Handler             /* 95: Reserved.                */
    .long   SPI0_IRQHandler             /* 96: SPI 0                    */
    .long   SPI1_IRQHandler             /* 97: SPI 1                    */
    .long   SPI2_IRQHandler             /* 98: SPI 2                    */
    .long   SPI3_IRQHandler             /* 99: SPI 3                    */
    .long   Default_Handler             /* 100: Reserved.               */
    .long   Default_Handler             /* 101: Reserved.               */
    .long   Default_Handler             /* 102: Reserved.               */
    .long   Default_Handler             /* 103: Reserved.               */
    .long   UART0_IRQHandler            /* 104: UART 0                  */
    .long   UART1_IRQHandler            /* 105: UART 1                  */
    .long   UART2_IRQHandler            /* 106: UART 2                  */
    .long   UART3_IRQHandler            /* 107: UART 3                  */
    .long   UART4_IRQHandler            /* 108: UART 4                  */
    .long   UART5_IRQHandler            /* 109: UART 5                  */
    .long   Default_Handler             /* 110: Reserved.               */
    .long   Default_Handler             /* 111: Reserved.               */
    .long   I2C0_IRQHandler             /* 112: I2C 0                   */
    .long   I2C1_IRQHandler             /* 113: I2C 1                   */
    .long   I2C2_IRQHandler             /* 114: I2C 2                   */
    .long   I2C3_IRQHandler             /* 115: I2C 3                   */
    .long   I2C4_IRQHandler             /* 116: I2C 4                   */
    .long   Default_Handler             /* 117: Reserved.               */
    .long   Default_Handler             /* 118: Reserved.               */
    .long   Default_Handler             /* 119: Reserved.               */
    .long   SC0_IRQHandler              /* 120: Smart Card 0            */
    .long   SC1_IRQHandler              /* 121: Smart Card 1            */
    .long   SC2_IRQHandler              /* 122: Smart Card 2            */
    .long   SC3_IRQHandler              /* 123: Smart Card 3            */
    .long   SC4_IRQHandler              /* 124: Smart Card 4            */
    .long   SC5_IRQHandler              /* 125: Smart Card 5            */
    .long   Default_Handler             /* 126: Reserved.               */
    .long   Default_Handler             /* 127: Reserved.               */
    .long   CAN0_IRQHandler             /* 128: CAN 0                   */
    .long   CAN1_IRQHandler             /* 129: CAN 1                   */
    .long   Default_Handler             /* 130: Reserved.               */
    .long   Default_Handler             /* 131: Reserved.               */
    .long   I2S0_IRQHandler             /* 132: I2S 0                   */
    .long   I2S1_IRQHandler             /* 133: I2S 1                   */
    .long   Default_Handler             /* 134: Reserved.               */
    .long   Default_Handler             /* 135: Reserved.               */
    .long   SD_IRQHandler               /* 136: SD card                 */
    .long   Default_Handler             /* 137: Reserved.               */
    .long   PS2D_IRQHandler             /* 138: PS/2 device             */
    .long   CAP_IRQHandler              /* 139: VIN                     */
    .long   CRYPTO_IRQHandler           /* 140: CRYPTO                  */
    .long   CRC_IRQHandler              /* 141: CRC                     */
                
   
    
    .size    __isr_vector, . - __isr_vector

    .section .text.Reset_Handler
    .thumb
    .thumb_func
    .align  2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    /* Unlock Register */
    LDR     R0, =0x40000100
    LDR     R1, =0x59
    STR     R1, [R0]
    LDR     R1, =0x16
    STR     R1, [R0]
    LDR     R1, =0x88
    STR     R1, [R0]
                
    /* Disable branch buffer if VCID is 0 */
    LDR     R2, =0x40000020
    LDR     R1, [R2]
    CMP     R1, #0
    BNE     Lock
    LDR     R2, =0x4000C018
    LDR     R1, [R2]
    ORR     R1, #0x80
    STR     R1, [R2]

Lock:
    /* Init POR */
    LDR     R2, =0x40000024
    LDR     R1, =0x00005AA5
    STR     R1, [R2]

    /* Lock register */
    MOVS    R1, #0
    STR     R1, [R0]
    
    /**
     * Because EBI (external SRAM) init is done in SystemInit(), SystemInit() must be called at the very start.
     */
    ldr   r0, =SystemInit
    blx   r0
    
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs   r3, r2
    ble    .Lflash_to_ram_loop_end

    movs    r4, 0
.Lflash_to_ram_loop:
    ldr    r0, [r1,r4]
    str    r0, [r2,r4]
    adds   r4, 4
    cmp    r4, r3
    blt    .Lflash_to_ram_loop
.Lflash_to_ram_loop_end:

    
    ldr   r0, =uvisor_init
    blx   r0
    ldr   r0, =_start
    bx    r0
    .pool
    .size Reset_Handler, . - Reset_Handler

    .text
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    /* Exception Handlers */
    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    UsageFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    DebugMon_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler
    
    .macro    def_irq_default_handler    handler_name
    .weak     \handler_name
    .set      \handler_name, Default_Handler
    .endm

    /* IRQ Handlers */
    def_irq_default_handler     BOD_IRQHandler          /* 0: Brown Out detection   */
    def_irq_default_handler     IRC_IRQHandler          /* 1: Internal RC           */
    def_irq_default_handler     PWRWU_IRQHandler        /* 2: Power Down Wake Up    */
    def_irq_default_handler     SRAMF_IRQHandler        /* 3: Reserved.             */
    def_irq_default_handler     CLKF_IRQHandler         /* 4: CLKF                  */
                                                        /* 5: Reserved.             */
    def_irq_default_handler     RTC_IRQHandler          /* 6: Real Time Clock       */
    def_irq_default_handler     TAMPER_IRQHandler       /* 7: Tamper detection      */
    def_irq_default_handler     EINT0_IRQHandler        /* 8: External Input 0      */
    def_irq_default_handler     EINT1_IRQHandler        /* 9: External Input 1      */
    def_irq_default_handler     EINT2_IRQHandler        /* 10: External Input 2     */
    def_irq_default_handler     EINT3_IRQHandler        /* 11: External Input 3     */
    def_irq_default_handler     EINT4_IRQHandler        /* 12: External Input 4     */
    def_irq_default_handler     EINT5_IRQHandler        /* 13: External Input 5     */
    def_irq_default_handler     EINT6_IRQHandler        /* 14: External Input 6     */
    def_irq_default_handler     EINT7_IRQHandler        /* 15: External Input 7     */
    def_irq_default_handler     GPA_IRQHandler          /* 16: GPIO Port A          */
    def_irq_default_handler     GPB_IRQHandler          /* 17: GPIO Port B          */
    def_irq_default_handler     GPC_IRQHandler          /* 18: GPIO Port C          */
    def_irq_default_handler     GPD_IRQHandler          /* 19: GPIO Port D          */
    def_irq_default_handler     GPE_IRQHandler          /* 20: GPIO Port E          */
    def_irq_default_handler     GPF_IRQHandler          /* 21: GPIO Port F          */
    def_irq_default_handler     GPG_IRQHandler          /* 22: GPIO Port G          */
    def_irq_default_handler     GPH_IRQHandler          /* 23: GPIO Port H          */
    def_irq_default_handler     GPI_IRQHandler          /* 24: GPIO Port I          */
                                                        /* 25: Reserved.            */
                                                        /* 26: Reserved.            */
                                                        /* 27: Reserved.            */
                                                        /* 28: Reserved.            */
                                                        /* 29: Reserved.            */
                                                        /* 30: Reserved.            */
                                                        /* 31: Reserved.            */
    def_irq_default_handler     TMR0_IRQHandler         /* 32: Timer 0              */
    def_irq_default_handler     TMR1_IRQHandler         /* 33: Timer 1              */
    def_irq_default_handler     TMR2_IRQHandler         /* 34: Timer 2              */
    def_irq_default_handler     TMR3_IRQHandler         /* 35: Timer 3              */
                                                        /* 36: Reserved.            */
                                                        /* 37: Reserved.            */
                                                        /* 38: Reserved.            */
                                                        /* 39: Reserved.            */
    def_irq_default_handler     PDMA_IRQHandler         /* 40: Peripheral DMA       */
                                                        /* 41: Reserved.            */
    def_irq_default_handler     ADC_IRQHandler          /* 42: ADC                  */
                                                        /* 43: Reserved.            */
                                                        /* 44: Reserved.            */
                                                        /* 45: Reserved.            */
    def_irq_default_handler     WDT_IRQHandler          /* 46: Watch Dog Timer      */
    def_irq_default_handler     WWDT_IRQHandler         /* 47: Window Watch Dog Timer   */
    def_irq_default_handler     EADC0_IRQHandler        /* 48: EDAC 0               */
    def_irq_default_handler     EADC1_IRQHandler        /* 49: EDAC 1               */
    def_irq_default_handler     EADC2_IRQHandler        /* 50: EDAC 2               */
    def_irq_default_handler     EADC3_IRQHandler        /* 51: EDAC 3               */
                                                        /* 52: Reserved.            */
                                                        /* 53: Reserved.            */
                                                        /* 54: Reserved.            */
                                                        /* 55: Reserved.            */
    def_irq_default_handler     ACMP_IRQHandler         /* 56: Analog Comparator    */
                                                        /* 57: Reserved.            */
                                                        /* 58: Reserved.            */
                                                        /* 59: Reserved.            */
    def_irq_default_handler     OPA0_IRQHandler         /* 60: OPA 0                */
    def_irq_default_handler     OPA1_IRQHandler         /* 61: OPA 1                */
    def_irq_default_handler     ICAP0_IRQHandler        /* 62: ICAP 0               */
    def_irq_default_handler     ICAP1_IRQHandler        /* 63: ICAP 1               */
    def_irq_default_handler     PWM0CH0_IRQHandler      /* 64: PWM0 CH0             */
    def_irq_default_handler     PWM0CH1_IRQHandler      /* 65: PWM0 CH1             */
    def_irq_default_handler     PWM0CH2_IRQHandler      /* 66: PWM0 CH2             */
    def_irq_default_handler     PWM0CH3_IRQHandler      /* 67: PWM0 CH3             */
    def_irq_default_handler     PWM0CH4_IRQHandler      /* 68: PWM0 CH4             */
    def_irq_default_handler     PWM0CH5_IRQHandler      /* 69: PWM0 CH5             */
    def_irq_default_handler     PWM0_BRK_IRQHandler     /* 70: PWM0 Break           */
    def_irq_default_handler     QEI0_IRQHandler         /* 71: QEI 0                */
    def_irq_default_handler     PWM1CH0_IRQHandler      /* 72: PWM1 CH0             */
    def_irq_default_handler     PWM1CH1_IRQHandler      /* 73: PWM1 CH1             */
    def_irq_default_handler     PWM1CH2_IRQHandler      /* 74: PWM1 CH2             */
    def_irq_default_handler     PWM1CH3_IRQHandler      /* 75: PWM1 CH3             */
    def_irq_default_handler     PWM1CH4_IRQHandler      /* 76: PWM1 CH4             */
    def_irq_default_handler     PWM1CH5_IRQHandler      /* 77: PWM1 CH5             */
    def_irq_default_handler     PWM1_BRK_IRQHandler     /* 78: PWM1 Break           */
    def_irq_default_handler     QEI1_IRQHandler         /* 79: QEI 1                */
    def_irq_default_handler     EPWM0_IRQHandler        /* 80: EPWM0                */
    def_irq_default_handler     EPWM0BRK_IRQHandler     /* 81: EPWM0 Break          */
    def_irq_default_handler     EPWM1_IRQHandler        /* 82: EPWM1                */
    def_irq_default_handler     EPWM1BRK_IRQHandler     /* 83: EPWM1 Break          */
                                                        /* 84: Reserved.            */
                                                        /* 85: Reserved.            */
                                                        /* 86: Reserved.            */
                                                        /* 87: Reserved.            */
    def_irq_default_handler     USBD_IRQHandler         /* 88: USB Device           */
    def_irq_default_handler     USBH_IRQHandler         /* 89: USB Host             */
    def_irq_default_handler     USB_OTG_IRQHandler      /* 90: USB OTG              */
                                                        /* 91: Reserved.            */
    def_irq_default_handler     EMAC_TX_IRQHandler      /* 92: Ethernet MAC TX      */
    def_irq_default_handler     EMAC_RX_IRQHandler      /* 93: Ethernet MAC RX      */
                                                        /* 94: Reserved.            */
                                                        /* 95: Reserved.            */
    def_irq_default_handler     SPI0_IRQHandler         /* 96: SPI 0                */
    def_irq_default_handler     SPI1_IRQHandler         /* 97: SPI 1                */
    def_irq_default_handler     SPI2_IRQHandler         /* 98: SPI 2                */
    def_irq_default_handler     SPI3_IRQHandler         /* 99: SPI 3                */
                                                        /* 100: Reserved.           */
                                                        /* 101: Reserved.           */
                                                        /* 102: Reserved.           */
                                                        /* 103: Reserved.           */
    def_irq_default_handler     UART0_IRQHandler        /* 104: UART 0              */
    def_irq_default_handler     UART1_IRQHandler        /* 105: UART 1              */
    def_irq_default_handler     UART2_IRQHandler        /* 106: UART 2              */
    def_irq_default_handler     UART3_IRQHandler        /* 107: UART 3              */
    def_irq_default_handler     UART4_IRQHandler        /* 108: UART 4              */
    def_irq_default_handler     UART5_IRQHandler        /* 109: UART 5              */
                                                        /* 110: Reserved.           */
                                                        /* 111: Reserved.           */
    def_irq_default_handler     I2C0_IRQHandler         /* 112: I2C 0               */
    def_irq_default_handler     I2C1_IRQHandler         /* 113: I2C 1               */
    def_irq_default_handler     I2C2_IRQHandler         /* 114: I2C 2               */
    def_irq_default_handler     I2C3_IRQHandler         /* 115: I2C 3               */
    def_irq_default_handler     I2C4_IRQHandler         /* 116: I2C 4               */
                                                        /* 117: Reserved.           */
                                                        /* 118: Reserved.           */
                                                        /* 119: Reserved.           */
    def_irq_default_handler     SC0_IRQHandler          /* 120: Smart Card 0        */
    def_irq_default_handler     SC1_IRQHandler          /* 121: Smart Card 1        */
    def_irq_default_handler     SC2_IRQHandler          /* 122: Smart Card 2        */
    def_irq_default_handler     SC3_IRQHandler          /* 123: Smart Card 3        */
    def_irq_default_handler     SC4_IRQHandler          /* 124: Smart Card 4        */
    def_irq_default_handler     SC5_IRQHandler          /* 125: Smart Card 5        */
                                                        /* 126: Reserved.           */
                                                        /* 127: Reserved.           */
    def_irq_default_handler     CAN0_IRQHandler         /* 128: CAN 0               */
    def_irq_default_handler     CAN1_IRQHandler         /* 129: CAN 1               */
                                                        /* 130: Reserved.           */
                                                        /* 131: Reserved.           */
    def_irq_default_handler     I2S0_IRQHandler         /* 132: I2S 0               */
    def_irq_default_handler     I2S1_IRQHandler         /* 133: I2S 1               */
                                                        /* 134: Reserved.           */
                                                        /* 135: Reserved.           */
    def_irq_default_handler     SD_IRQHandler           /* 136: SD card             */
                                                        /* 137: Reserved.           */
    def_irq_default_handler     PS2D_IRQHandler         /* 138: PS/2 device         */
    def_irq_default_handler     CAP_IRQHandler          /* 139: VIN                 */
    def_irq_default_handler     CRYPTO_IRQHandler       /* 140: CRYPTO              */
    def_irq_default_handler     CRC_IRQHandler          /* 141: CRC                 */
    

    .end
