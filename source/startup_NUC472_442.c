/******************************************************************************
 * @file     startup_NUC472_442.c
 * @version  V0.10
 * $Revision: 11 $
 * $Date: 15/09/02 10:02a $
 * @brief    CMSIS Cortex-M4 Core Peripheral Access Layer Source File for NUC472/442 MCU
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NUC472_442.h"

/* Macro Definitions */
#if defined(__CC_ARM)
#define WEAK            __attribute__ ((weak))
#define ALIAS(f)        __attribute__ ((weak, alias(#f)))

#elif defined(__ICCARM__)

#elif defined(__GNUC__)
#define WEAK            __attribute__ ((weak))
#define ALIAS(f)        __attribute__ ((weak, alias(#f)))

#endif


/* Initialize segments */
#if defined(__CC_ARM)
extern uint32_t Image$$ARM_LIB_STACK$$ZI$$Limit;
extern void __main(void);
#elif defined(__ICCARM__)

#elif defined(__GNUC__)
extern uint32_t __StackTop;
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_extern_start__ WEAK;
extern uint32_t __bss_extern_end__ WEAK;

extern void uvisor_init(void);
extern void _start(void);
#endif

/* Default empty handler */
void Default_Handler(void);

/* Reset handler */
void Reset_Handler(void);

/* Cortex-M4 core handlers */
void NMI_Handler(void)              ALIAS(Default_Handler);
void HardFault_Handler(void)        ALIAS(Default_Handler);
void MemManage_Handler(void)        ALIAS(Default_Handler);
void BusFault_Handler (void)        ALIAS(Default_Handler);
void UsageFault_Handler(void)       ALIAS(Default_Handler);
void SVC_Handler(void)              ALIAS(Default_Handler);
void DebugMon_Handler(void)         ALIAS(Default_Handler);
void PendSV_Handler(void)           ALIAS(Default_Handler);
void SysTick_Handler(void)          ALIAS(Default_Handler);

/* Peripherals handlers */
void BOD_IRQHandler(void)           ALIAS(Default_Handler);     // 0: Brown Out detection
void IRC_IRQHandler(void)           ALIAS(Default_Handler);     // 1: Internal RC
void PWRWU_IRQHandler(void)         ALIAS(Default_Handler);     // 2: Power Down Wake Up 
void SRAMF_IRQHandler(void)         ALIAS(Default_Handler);     // 3: Reserved.
void CLKF_IRQHandler(void)          ALIAS(Default_Handler);     // 4: CLKF
// 5: Reserved.
void RTC_IRQHandler(void)           ALIAS(Default_Handler);     // 6: Real Time Clock 
void TAMPER_IRQHandler(void)        ALIAS(Default_Handler);     // 7: Tamper detection
void EINT0_IRQHandler(void)         ALIAS(Default_Handler);     // 8: External Input 0
void EINT1_IRQHandler(void)         ALIAS(Default_Handler);     // 9: External Input 1
void EINT2_IRQHandler(void)         ALIAS(Default_Handler);     // 10: External Input 2
void EINT3_IRQHandler(void)         ALIAS(Default_Handler);     // 11: External Input 3
void EINT4_IRQHandler(void)         ALIAS(Default_Handler);     // 12: External Input 4
void EINT5_IRQHandler(void)         ALIAS(Default_Handler);     // 13: External Input 5
void EINT6_IRQHandler(void)         ALIAS(Default_Handler);     // 14: External Input 6
void EINT7_IRQHandler(void)         ALIAS(Default_Handler);     // 15: External Input 7 
void GPA_IRQHandler(void)           ALIAS(Default_Handler);     // 16: GPIO Port A
void GPB_IRQHandler(void)           ALIAS(Default_Handler);     // 17: GPIO Port B
void GPC_IRQHandler(void)           ALIAS(Default_Handler);     // 18: GPIO Port C
void GPD_IRQHandler(void)           ALIAS(Default_Handler);     // 19: GPIO Port D
void GPE_IRQHandler(void)           ALIAS(Default_Handler);     // 20: GPIO Port E
void GPF_IRQHandler(void)           ALIAS(Default_Handler);     // 21: GPIO Port F
void GPG_IRQHandler(void)           ALIAS(Default_Handler);     // 22: GPIO Port G
void GPH_IRQHandler(void)           ALIAS(Default_Handler);     // 23: GPIO Port H
void GPI_IRQHandler(void)           ALIAS(Default_Handler);     // 24: GPIO Port I
                                                                // 25: Reserved.
                                                                // 26: Reserved.
                                                                // 27: Reserved.
                                                                // 28: Reserved.
                                                                // 29: Reserved.
                                                                // 30: Reserved.
                                                                // 31: Reserved.
void TMR0_IRQHandler(void)          ALIAS(Default_Handler);     // 32: Timer 0
void TMR1_IRQHandler(void)          ALIAS(Default_Handler);     // 33: Timer 1
void TMR2_IRQHandler(void)          ALIAS(Default_Handler);     // 34: Timer 2
void TMR3_IRQHandler(void)          ALIAS(Default_Handler);     // 35: Timer 3
                                                                // 36: Reserved.
                                                                // 37: Reserved.
                                                                // 38: Reserved.
                                                                // 39: Reserved.
void PDMA_IRQHandler(void)          ALIAS(Default_Handler);     // 40: Peripheral DMA
                                                                // 41: Reserved.
void ADC_IRQHandler(void)           ALIAS(Default_Handler);     // 42: ADC
                                                                // 43: Reserved.
                                                                // 44: Reserved.
                                                                // 45: Reserved.
void WDT_IRQHandler(void)           ALIAS(Default_Handler);     // 46: Watch Dog Timer
void WWDT_IRQHandler(void)          ALIAS(Default_Handler);     // 47: Window Watch Dog Timer
void EADC0_IRQHandler(void)         ALIAS(Default_Handler);     // 48: EDAC 0
void EADC1_IRQHandler(void)         ALIAS(Default_Handler);     // 49: EDAC 1
void EADC2_IRQHandler(void)         ALIAS(Default_Handler);     // 50: EDAC 2
void EADC3_IRQHandler(void)         ALIAS(Default_Handler);     // 51: EDAC 3
                                                                // 52: Reserved.
                                                                // 53: Reserved.
                                                                // 54: Reserved.
                                                                // 55: Reserved.
void ACMP_IRQHandler(void)          ALIAS(Default_Handler);     // 56: Analog Comparator
                                                                // 57: Reserved.
                                                                // 58: Reserved.
                                                                // 59: Reserved.
void OPA0_IRQHandler(void)          ALIAS(Default_Handler);     // 60: OPA 0
void OPA1_IRQHandler(void)          ALIAS(Default_Handler);     // 61: OPA 1
void ICAP0_IRQHandler(void)         ALIAS(Default_Handler);     // 62: ICAP 0
void ICAP1_IRQHandler(void)         ALIAS(Default_Handler);     // 63: ICAP 1
void PWM0CH0_IRQHandler(void)       ALIAS(Default_Handler);     // 64: PWM0 CH0
void PWM0CH1_IRQHandler(void)       ALIAS(Default_Handler);     // 65: PWM0 CH1
void PWM0CH2_IRQHandler(void)       ALIAS(Default_Handler);     // 66: PWM0 CH2
void PWM0CH3_IRQHandler(void)       ALIAS(Default_Handler);     // 67: PWM0 CH3
void PWM0CH4_IRQHandler(void)       ALIAS(Default_Handler);     // 68: PWM0 CH4
void PWM0CH5_IRQHandler(void)       ALIAS(Default_Handler);     // 69: PWM0 CH5
void PWM0_BRK_IRQHandler(void)      ALIAS(Default_Handler);     // 70: PWM0 Break
void QEI0_IRQHandler(void)          ALIAS(Default_Handler);     // 71: QEI 0
void PWM1CH0_IRQHandler(void)       ALIAS(Default_Handler);     // 72: PWM1 CH0
void PWM1CH1_IRQHandler(void)       ALIAS(Default_Handler);     // 73: PWM1 CH1
void PWM1CH2_IRQHandler(void)       ALIAS(Default_Handler);     // 74: PWM1 CH2
void PWM1CH3_IRQHandler(void)       ALIAS(Default_Handler);     // 75: PWM1 CH3
void PWM1CH4_IRQHandler(void)       ALIAS(Default_Handler);     // 76: PWM1 CH4
void PWM1CH5_IRQHandler(void)       ALIAS(Default_Handler);     // 77: PWM1 CH5
void PWM1_BRK_IRQHandler(void)      ALIAS(Default_Handler);     // 78: PWM1 Break
void QEI1_IRQHandler(void)          ALIAS(Default_Handler);     // 79: QEI 1
void EPWM0_IRQHandler(void)         ALIAS(Default_Handler);     // 80: EPWM0
void EPWM0BRK_IRQHandler(void)      ALIAS(Default_Handler);     // 81: EPWM0 Break
void EPWM1_IRQHandler(void)         ALIAS(Default_Handler);     // 82: EPWM1
void EPWM1BRK_IRQHandler(void)      ALIAS(Default_Handler);     // 83: EPWM1 Break
                                                                // 84: Reserved.
                                                                // 85: Reserved.
                                                                // 86: Reserved.
                                                                // 87: Reserved.
void USBD_IRQHandler(void)          ALIAS(Default_Handler);     // 88: USB Device
void USBH_IRQHandler(void)          ALIAS(Default_Handler);     // 89: USB Host
void USB_OTG_IRQHandler(void)       ALIAS(Default_Handler);     // 90: USB OTG
                                                                // 91: Reserved.
void EMAC_TX_IRQHandler(void)       ALIAS(Default_Handler);     // 92: Ethernet MAC TX
void EMAC_RX_IRQHandler(void)       ALIAS(Default_Handler);     // 93: Ethernet MAC RX
                                                                // 94: Reserved.
                                                                // 95: Reserved.
void SPI0_IRQHandler(void)          ALIAS(Default_Handler);     // 96: SPI 0
void SPI1_IRQHandler(void)          ALIAS(Default_Handler);     // 97: SPI 1
void SPI2_IRQHandler(void)          ALIAS(Default_Handler);     // 98: SPI 2
void SPI3_IRQHandler(void)          ALIAS(Default_Handler);     // 99: SPI 3
                                                                // 100: Reserved.
                                                                // 101: Reserved.
                                                                // 102: Reserved.
                                                                // 103: Reserved.
void UART0_IRQHandler(void)         ALIAS(Default_Handler);     // 104: UART 0
void UART1_IRQHandler(void)         ALIAS(Default_Handler);     // 105: UART 1
void UART2_IRQHandler(void)         ALIAS(Default_Handler);     // 106: UART 2
void UART3_IRQHandler(void)         ALIAS(Default_Handler);     // 107: UART 3
void UART4_IRQHandler(void)         ALIAS(Default_Handler);     // 108: UART 4
void UART5_IRQHandler(void)         ALIAS(Default_Handler);     // 109: UART 5
                                                                // 110: Reserved.
                                                                // 111: Reserved.
void I2C0_IRQHandler(void)          ALIAS(Default_Handler);     // 112: I2C 0
void I2C1_IRQHandler(void)          ALIAS(Default_Handler);     // 113: I2C 1
void I2C2_IRQHandler(void)          ALIAS(Default_Handler);     // 114: I2C 2
void I2C3_IRQHandler(void)          ALIAS(Default_Handler);     // 115: I2C 3
void I2C4_IRQHandler(void)          ALIAS(Default_Handler);     // 116: I2C 4
                                                                // 117: Reserved.
                                                                // 118: Reserved.
                                                                // 119: Reserved.
void SC0_IRQHandler(void)           ALIAS(Default_Handler);     // 120: Smart Card 0
void SC1_IRQHandler(void)           ALIAS(Default_Handler);     // 121: Smart Card 1
void SC2_IRQHandler(void)           ALIAS(Default_Handler);     // 122: Smart Card 2
void SC3_IRQHandler(void)           ALIAS(Default_Handler);     // 123: Smart Card 3
void SC4_IRQHandler(void)           ALIAS(Default_Handler);     // 124: Smart Card 4
void SC5_IRQHandler(void)           ALIAS(Default_Handler);     // 125: Smart Card 5
                                                                // 126: Reserved.
                                                                // 127: Reserved.
void CAN0_IRQHandler(void)          ALIAS(Default_Handler);     // 128: CAN 0
void CAN1_IRQHandler(void)          ALIAS(Default_Handler);     // 129: CAN 1
                                                                // 130: Reserved.
                                                                // 131: Reserved.
void I2S0_IRQHandler(void)          ALIAS(Default_Handler);     // 132: I2S 0
void I2S1_IRQHandler(void)          ALIAS(Default_Handler);     // 133: I2S 1
                                                                // 134: Reserved.
                                                                // 135: Reserved.
void SD_IRQHandler(void)            ALIAS(Default_Handler);     // 136: SD card
                                                                // 137: Reserved.
void PS2D_IRQHandler(void)          ALIAS(Default_Handler);     // 138: PS/2 device
void CAP_IRQHandler(void)           ALIAS(Default_Handler);     // 139: VIN
void CRYPTO_IRQHandler(void)        ALIAS(Default_Handler);     // 140: CRYPTO
void CRC_IRQHandler(void)           ALIAS(Default_Handler);     // 141: CRC

/* Vector table */
#if defined(__CC_ARM)
__attribute__ ((section("RESET")))
#elif defined(__ICCARM__)
#elif defined(__GNUC__)
__attribute__ ((section(".vector_table")))
#endif
const uint32_t __vector_handlers[] = {

    /* Configure Initial Stack Pointer, using linker-generated symbols */
#if defined(__CC_ARM)
    &Image$$ARM_LIB_STACK$$ZI$$Limit,
#elif defined(__ICCARM__)
    0, /* Reserved */
#elif defined(__GNUC__)
    &__StackTop,
#endif

    Reset_Handler,          // Reset Handler
    NMI_Handler,            // NMI Handler
    HardFault_Handler,      // Hard Fault Handler
    MemManage_Handler,      // MPU Fault Handler
    BusFault_Handler,       // Bus Fault Handler
    UsageFault_Handler,     // Usage Fault Handler
    0,                      // Reserved
    0,                      // Reserved
    0,                      // Reserved
    0,                      // Reserved
    SVC_Handler,            // SVCall Handler
    DebugMon_Handler,       // Debug Monitor Handler
    0,                      // Reserved
    PendSV_Handler,         // PendSV Handler
    SysTick_Handler,        // SysTick Handler

    /* External Interrupts */
    BOD_IRQHandler,         // 0: Brown Out detection
    IRC_IRQHandler,         // 1: Internal RC
    PWRWU_IRQHandler,       // 2: Power Down Wake Up 
    SRAMF_IRQHandler,       // 3: Reserved.
    CLKF_IRQHandler,        // 4: CLKF
    Default_Handler,        // 5: Reserved.
    RTC_IRQHandler,         // 6: Real Time Clock 
    TAMPER_IRQHandler,      // 7: Tamper detection
    EINT0_IRQHandler,       // 8: External Input 0
    EINT1_IRQHandler,       // 9: External Input 1
    EINT2_IRQHandler,       // 10: External Input 2
    EINT3_IRQHandler,       // 11: External Input 3
    EINT4_IRQHandler,       // 12: External Input 4
    EINT5_IRQHandler,       // 13: External Input 5
    EINT6_IRQHandler,       // 14: External Input 6
    EINT7_IRQHandler,       // 15: External Input 7 
    GPA_IRQHandler,         // 16: GPIO Port A
    GPB_IRQHandler,         // 17: GPIO Port B
    GPC_IRQHandler,         // 18: GPIO Port C
    GPD_IRQHandler,         // 19: GPIO Port D
    GPE_IRQHandler,         // 20: GPIO Port E
    GPF_IRQHandler,         // 21: GPIO Port F
    GPG_IRQHandler,         // 22: GPIO Port G
    GPH_IRQHandler,         // 23: GPIO Port H
    GPI_IRQHandler,         // 24: GPIO Port I
    Default_Handler,        // 25: Reserved.
    Default_Handler,        // 26: Reserved.
    Default_Handler,        // 27: Reserved.
    Default_Handler,        // 28: Reserved.
    Default_Handler,        // 29: Reserved.
    Default_Handler,        // 30: Reserved.
    Default_Handler,        // 31: Reserved.
    TMR0_IRQHandler,        // 32: Timer 0
    TMR1_IRQHandler,        // 33: Timer 1
    TMR2_IRQHandler,        // 34: Timer 2
    TMR3_IRQHandler,        // 35: Timer 3
    Default_Handler,        // 36: Reserved.
    Default_Handler,        // 37: Reserved.
    Default_Handler,        // 38: Reserved.
    Default_Handler,        // 39: Reserved.
    PDMA_IRQHandler,        // 40: Peripheral DMA
    Default_Handler,        // 41: Reserved.
    ADC_IRQHandler,         // 42: ADC
    Default_Handler,        // 43: Reserved.
    Default_Handler,        // 44: Reserved.
    Default_Handler,        // 45: Reserved.
    WDT_IRQHandler,         // 46: Watch Dog Timer
    WWDT_IRQHandler,        // 47: Window Watch Dog Timer
    EADC0_IRQHandler,       // 48: EDAC 0
    EADC1_IRQHandler,       // 49: EDAC 1
    EADC2_IRQHandler,       // 50: EDAC 2
    EADC3_IRQHandler,       // 51: EDAC 3
    Default_Handler,        // 52: Reserved.
    Default_Handler,        // 53: Reserved.
    Default_Handler,        // 54: Reserved.
    Default_Handler,        // 55: Reserved.
    ACMP_IRQHandler,        // 56: Analog Comparator
    Default_Handler,        // 57: Reserved.
    Default_Handler,        // 58: Reserved.
    Default_Handler,        // 59: Reserved.
    OPA0_IRQHandler,        // 60: OPA 0
    OPA1_IRQHandler,        // 61: OPA 1
    ICAP0_IRQHandler,       // 62: ICAP 0
    ICAP1_IRQHandler,       // 63: ICAP 1
    PWM0CH0_IRQHandler,     // 64: PWM0 CH0
    PWM0CH1_IRQHandler,     // 65: PWM0 CH1
    PWM0CH2_IRQHandler,     // 66: PWM0 CH2
    PWM0CH3_IRQHandler,     // 67: PWM0 CH3
    PWM0CH4_IRQHandler,     // 68: PWM0 CH4
    PWM0CH5_IRQHandler,     // 69: PWM0 CH5
    PWM0_BRK_IRQHandler,    // 70: PWM0 Break
    QEI0_IRQHandler,        // 71: QEI 0
    PWM1CH0_IRQHandler,     // 72: PWM1 CH0
    PWM1CH1_IRQHandler,     // 73: PWM1 CH1
    PWM1CH2_IRQHandler,     // 74: PWM1 CH2
    PWM1CH3_IRQHandler,     // 75: PWM1 CH3
    PWM1CH4_IRQHandler,     // 76: PWM1 CH4
    PWM1CH5_IRQHandler,     // 77: PWM1 CH5
    PWM1_BRK_IRQHandler,    // 78: PWM1 Break
    QEI1_IRQHandler,        // 79: QEI 1
    EPWM0_IRQHandler,       // 80: EPWM0
    EPWM0BRK_IRQHandler,    // 81: EPWM0 Break
    EPWM1_IRQHandler,       // 82: EPWM1
    EPWM1BRK_IRQHandler,    // 83: EPWM1 Break
    Default_Handler,        // 84: Reserved.
    Default_Handler,        // 85: Reserved.
    Default_Handler,        // 86: Reserved.
    Default_Handler,        // 87: Reserved.
    USBD_IRQHandler,        // 88: USB Device
    USBH_IRQHandler,        // 89: USB Host
    USB_OTG_IRQHandler,     // 90: USB OTG
    Default_Handler,        // 91: Reserved.
    EMAC_TX_IRQHandler,     // 92: Ethernet MAC TX
    EMAC_RX_IRQHandler,     // 93: Ethernet MAC RX
    Default_Handler,        // 94: Reserved.
    Default_Handler,        // 95: Reserved.
    SPI0_IRQHandler,        // 96: SPI 0
    SPI1_IRQHandler,        // 97: SPI 1
    SPI2_IRQHandler,        // 98: SPI 2
    SPI3_IRQHandler,        // 99: SPI 3
    Default_Handler,        // 100: Reserved.
    Default_Handler,        // 101: Reserved.
    Default_Handler,        // 102: Reserved.
    Default_Handler,        // 103: Reserved.
    UART0_IRQHandler,       // 104: UART 0
    UART1_IRQHandler,       // 105: UART 1
    UART2_IRQHandler,       // 106: UART 2
    UART3_IRQHandler,       // 107: UART 3
    UART4_IRQHandler,       // 108: UART 4
    UART5_IRQHandler,       // 109: UART 5
    Default_Handler,        // 110: Reserved.
    Default_Handler,        // 111: Reserved.
    I2C0_IRQHandler,        // 112: I2C 0
    I2C1_IRQHandler,        // 113: I2C 1
    I2C2_IRQHandler,        // 114: I2C 2
    I2C3_IRQHandler,        // 115: I2C 3
    I2C4_IRQHandler,        // 116: I2C 4
    Default_Handler,        // 117: Reserved.
    Default_Handler,        // 118: Reserved.
    Default_Handler,        // 119: Reserved.
    SC0_IRQHandler,         // 120: Smart Card 0
    SC1_IRQHandler,         // 121: Smart Card 1
    SC2_IRQHandler,         // 122: Smart Card 2
    SC3_IRQHandler,         // 123: Smart Card 3
    SC4_IRQHandler,         // 124: Smart Card 4
    SC5_IRQHandler,         // 125: Smart Card 5
    Default_Handler,        // 126: Reserved.
    Default_Handler,        // 127: Reserved.
    CAN0_IRQHandler,        // 128: CAN 0
    CAN1_IRQHandler,        // 129: CAN 1
    Default_Handler,        // 130: Reserved.
    Default_Handler,        // 131: Reserved.
    I2S0_IRQHandler,        // 132: I2S 0
    I2S1_IRQHandler,        // 133: I2S 1
    Default_Handler,        // 134: Reserved.
    Default_Handler,        // 135: Reserved.
    SD_IRQHandler,          // 136: SD card
    Default_Handler,        // 137: Reserved.
    PS2D_IRQHandler,        // 138: PS/2 device
    CAP_IRQHandler,         // 139: VIN
    CRYPTO_IRQHandler,      // 140: CRYPTO
    CRC_IRQHandler,         // 141: CRC    
};

/**
 * \brief This is the code that gets called on processor reset.
 */
void Reset_Handler(void)
{
    /* Disable register write-protection function */
    SYS_UnlockReg();
    
    /* Disable branch buffer if VCID is 0 */
    if (SYS->VCID == 0) {
        FMC->FTCTL |= 0x80;
    }
    
    /* Disable Power-on Reset function */
    SYS_DISABLE_POR();
    
    /* Enable register write-protection function */
    SYS_LockReg();
    
    /**
     * Because EBI (external SRAM) init is done in SystemInit(), SystemInit() must be called at the very start.
     */
    SystemInit();
    
#if defined(__CC_ARM)
    __main();
    
#elif defined(__ICCARM__)

#elif defined(__GNUC__)
    uint32_t *src_ind = (uint32_t *) &__etext;
    uint32_t *dst_ind = (uint32_t *) &__data_start__;
    uint32_t *dst_end = (uint32_t *) &__data_end__;

    /* Move .data section from ROM to RAM */
    if (src_ind != dst_ind) {
        for (; dst_ind < dst_end;) {
            *dst_ind ++ = *src_ind ++;
        }
    }
   
    /* Initialize .bss.extern section to zero */
    dst_ind = (uint32_t *) &__bss_extern_start__;
    dst_end = (uint32_t *) &__bss_extern_end__;
    if (dst_ind != dst_end) {
        for (; dst_ind < dst_end;) {
            *dst_ind ++ = 0;
        }
    }
    
    uvisor_init();
    _start();
#endif

    /* Infinite loop */
    while (1);
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Default_Handler(void)
{
    while (1);
}
