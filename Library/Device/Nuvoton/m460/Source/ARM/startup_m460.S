/**************************************************************************//**
 * @file     startup_m460.s
 * @version  V3.00
 * @brief    M460 Series Startup Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/



    .section .bss.STACK, "aw", %nobits
    .align 3
    .global __initial_sp
#ifndef Stack_Size
// <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Stack_Size, 0x00001000
#endif
Stack_Mem:
    .space   Stack_Size
__initial_sp:


    .section .bss.HEAP, "aw", %nobits
    .align  3
    .global Heap_Mem
    .global __heap_base
    .global __heap_limit
#ifndef Heap_Size
// <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Heap_Size, 0x00000100
#endif
__heap_base:
Heap_Mem:
    .space  Heap_Size
__heap_limit:

    .eabi_attribute Tag_ABI_align_preserved, 1
    .thumb


// ; Vector Table Mapped to Address 0 at Reset
    .section RESET, "ax"
    .global     __Vectors
    .global     __Vectors_End
    .global     __Vectors_Size

__Vectors:
    .word     __initial_sp              // ; Top of Stack
    .word     Reset_Handler             // ; Reset Handler
    .word     NMI_Handler               // ; NMI Handler
    .word     HardFault_Handler         // ; Hard Fault Handler
    .word     MemManage_Handler         // ; MPU Fault Handler
    .word     BusFault_Handler          // ; Bus Fault Handler
    .word     UsageFault_Handler        // ; Usage Fault Handler
    .word     0                         // ; Reserved
    .word     0                         // ; Reserved
    .word     0                         // ; Reserved
    .word     0                         // ; Reserved
    .word     SVC_Handler               // ; SVCall Handler
    .word     DebugMon_Handler          // ; Debug Monitor Handler
    .word     0                         // ; Reserved
    .word     PendSV_Handler            // ; PendSV Handler
    .word     SysTick_Handler           // ; SysTick Hand

    // // ; External Interrupts
    .word     BOD_IRQHandler            // ; 0: Brown Out detection
    .word     IRC_IRQHandler            // ; 1: Internal RC
    .word     PWRWU_IRQHandler          // ; 2: Power down wake up
    .word     RAMPE_IRQHandler          // ; 3: RAM parity error
    .word     CKFAIL_IRQHandler         // ; 4: Clock detection fail
    .word     ISP_IRQHandler            // ; 5: ISP
    .word     RTC_IRQHandler            // ; 6: Real Time Clock
    .word     TAMPER_IRQHandler         // ; 7: Tamper detection
    .word     WDT_IRQHandler            // ; 8: Watchdog timer
    .word     WWDT_IRQHandler           // ; 9: Window watchdog timer
    .word     EINT0_IRQHandler          // ; 10: External Input 0
    .word     EINT1_IRQHandler          // ; 11: External Input 1
    .word     EINT2_IRQHandler          // ; 12: External Input 2
    .word     EINT3_IRQHandler          // ; 13: External Input 3
    .word     EINT4_IRQHandler          // ; 14: External Input 4
    .word     EINT5_IRQHandler          // ; 15: External Input 5
    .word     GPA_IRQHandler            // ; 16: GPIO Port A
    .word     GPB_IRQHandler            // ; 17: GPIO Port B
    .word     GPC_IRQHandler            // ; 18: GPIO Port C
    .word     GPD_IRQHandler            // ; 19: GPIO Port D
    .word     GPE_IRQHandler            // ; 20: GPIO Port E
    .word     GPF_IRQHandler            // ; 21: GPIO Port F
    .word     QSPI0_IRQHandler          // ; 22: QSPI0
    .word     SPI0_IRQHandler           // ; 23: SPI0
    .word     BRAKE0_IRQHandler         // ; 24: EPWM0 brake
    .word     EPWM0P0_IRQHandler        // ; 25: EPWM0 pair 0
    .word     EPWM0P1_IRQHandler        // ; 26: EPWM0 pair 1
    .word     EPWM0P2_IRQHandler        // ; 27: EPWM0 pair 2
    .word     BRAKE1_IRQHandler         // ; 28: EPWM1 brake
    .word     EPWM1P0_IRQHandler        // ; 29: EPWM1 pair 0
    .word     EPWM1P1_IRQHandler        // ; 30: EPWM1 pair 1
    .word     EPWM1P2_IRQHandler        // ; 31: EPWM1 pair 2
    .word     TMR0_IRQHandler           // ; 32: Timer 0
    .word     TMR1_IRQHandler           // ; 33: Timer 1
    .word     TMR2_IRQHandler           // ; 34: Timer 2
    .word     TMR3_IRQHandler           // ; 35: Timer 3
    .word     UART0_IRQHandler          // ; 36: UART0
    .word     UART1_IRQHandler          // ; 37: UART1
    .word     I2C0_IRQHandler           // ; 38: I2C0
    .word     I2C1_IRQHandler           // ; 39: I2C1
    .word     PDMA0_IRQHandler          // ; 40: Peripheral DMA 0
    .word     DAC_IRQHandler            // ; 41: DAC
    .word     EADC00_IRQHandler         // ; 42: EADC0 interrupt source 0
    .word     EADC01_IRQHandler         // ; 43: EADC0 interrupt source 1
    .word     ACMP01_IRQHandler         // ; 44: ACMP0 and ACMP1
    .word     ACMP23_IRQHandler         // ; 45: ACMP2 and ACMP3
    .word     EADC02_IRQHandler         // ; 46: EADC0 interrupt source 2
    .word     EADC03_IRQHandler         // ; 47: EADC0 interrupt source 3
    .word     UART2_IRQHandler          // ; 48: UART2
    .word     UART3_IRQHandler          // ; 49: UART3
    .word     QSPI1_IRQHandler          // ; 50: QSPI1
    .word     SPI1_IRQHandler           // ; 51: SPI1
    .word     SPI2_IRQHandler           // ; 52: SPI2
    .word     USBD_IRQHandler           // ; 53: USB device
    .word     OHCI_IRQHandler           // ; 54: OHCI
    .word     USBOTG_IRQHandler         // ; 55: USB OTG
    .word     BMC_IRQHandler            // ; 56: BMC
    .word     SPI5_IRQHandler           // ; 57: SPI5
    .word     SC0_IRQHandler            // ; 58: SC0
    .word     SC1_IRQHandler            // ; 59: SC1
    .word     SC2_IRQHandler            // ; 60: SC2
    .word     GPJ_IRQHandler            // ; 61: GPIO Port J
    .word     SPI3_IRQHandler           // ; 62: SPI3
    .word     SPI4_IRQHandler           // ; 63: SPI4
    .word     SDH0_IRQHandler           // ; 64: SDH0
    .word     USBD20_IRQHandler         // ; 65: USBD20
    .word     EMAC0_IRQHandler          // ; 66: EMAC0
    .word     Default_Handler           // ; 67:
    .word     I2S0_IRQHandler           // ; 68: I2S0
    .word     I2S1_IRQHandler           // ; 69: I2S1
    .word     SPI6_IRQHandler           // ; 70: SPI6
    .word     CRPT_IRQHandler           // ; 71: CRYPTO
    .word     GPG_IRQHandler            // ; 72: GPIO Port G
    .word     EINT6_IRQHandler          // ; 73: External Input 6
    .word     UART4_IRQHandler          // ; 74: UART4
    .word     UART5_IRQHandler          // ; 75: UART5
    .word     USCI0_IRQHandler          // ; 76: USCI0
    .word     SPI7_IRQHandler           // ; 77: SPI7
    .word     BPWM0_IRQHandler          // ; 78: BPWM0
    .word     BPWM1_IRQHandler          // ; 79: BPWM1
    .word     SPIM_IRQHandler           // ; 80: SPIM
    .word     CCAP_IRQHandler           // ; 81: CCAP
    .word     I2C2_IRQHandler           // ; 82: I2C2
    .word     I2C3_IRQHandler           // ; 83: I2C3
    .word     EQEI0_IRQHandler          // ; 84: EQEI0
    .word     EQEI1_IRQHandler          // ; 85: EQEI1
    .word     ECAP0_IRQHandler          // ; 86: ECAP0
    .word     ECAP1_IRQHandler          // ; 87: ECAP1
    .word     GPH_IRQHandler            // ; 88: GPIO Port H
    .word     EINT7_IRQHandler          // ; 89: External Input 7
    .word     SDH1_IRQHandler           // ; 90: SDH1
    .word     PSIO_IRQHandler           // ; 91: PSIO
    .word     EHCI_IRQHandler           // ; 92: EHCI
    .word     USBOTG20_IRQHandler       // ; 93: HSOTG
    .word     ECAP2_IRQHandler          // ; 94: ECAP2
    .word     ECAP3_IRQHandler          // ; 95: ECAP3
    .word     KPI_IRQHandler            // ; 96: KPI
    .word     HBI_IRQHandler            // ; 97: HBI
    .word     PDMA1_IRQHandler          // ; 98: Peripheral DMA 1
    .word     UART8_IRQHandler          // ; 99: UART8
    .word     UART9_IRQHandler          // ; 100: UART9
    .word     TRNG_IRQHandler           // ; 101: TRNG
    .word     UART6_IRQHandler          // ; 102: UART6
    .word     UART7_IRQHandler          // ; 103: UART7
    .word     EADC10_IRQHandler         // ; 104: EADC1 interrupt source 0
    .word     EADC11_IRQHandler         // ; 105: EADC1 interrupt source 1
    .word     EADC12_IRQHandler         // ; 106: EADC1 interrupt source 2
    .word     EADC13_IRQHandler         // ; 107: EADC1 interrupt source 3
    .word     SPI8_IRQHandler           // ; 108: SPI8
    .word     KS_IRQHandler             // ; 109: Key Store
    .word     GPI_IRQHandler            // ; 110: GPIO Port I
    .word     SPI9_IRQHandler           // ; 111: SPI9
    .word     CANFD00_IRQHandler        // ; 112: CANFD0 interrupt source 0
    .word     CANFD01_IRQHandler        // ; 113: CANFD0 interrupt source 1
    .word     CANFD10_IRQHandler        // ; 114: CANFD1 interrupt source 0
    .word     CANFD11_IRQHandler        // ; 115: CANFD1 interrupt source 1
    .word     EQEI2_IRQHandler          // ; 116: EQEI2
    .word     EQEI3_IRQHandler          // ; 117: EQEI3
    .word     I2C4_IRQHandler           // ; 118: I2C4
    .word     SPI10_IRQHandler          // ; 119: SPI10
    .word     CANFD20_IRQHandler        // ; 120: CANFD2 interrupt source 0
    .word     CANFD21_IRQHandler        // ; 121: CANFD2 interrupt source 1
    .word     CANFD30_IRQHandler        // ; 122: CANFD3 interrupt source 0
    .word     CANFD31_IRQHandler        // ; 123: CANFD3 interrupt source 1
    .word     EADC20_IRQHandler         // ; 124: EADC2 interrupt source 0
    .word     EADC21_IRQHandler         // ; 125: EADC2 interrupt source 1
    .word     EADC22_IRQHandler         // ; 126: EADC2 interrupt source 2
    .word     EADC23_IRQHandler         // ; 127: EADC2 interrupt source 3


__Vectors_End:
    .equ    __Vectors_Size, __Vectors_End - __Vectors

    .section .text, "ax"

// ; Reset Handler

    .global Reset_Handler
    .global  SystemInit
    .global  __main
    .type   Reset_Handler, "function"
Reset_Handler:
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__main
        BX      R0


// ; Dummy Exception Handlers (infinite loops which can be modified)

    .weak   NMI_Handler
    .type   NMI_Handler, "function"
NMI_Handler:
        B       .

    .weak   HardFault_Handler
    .type   HardFault_Handler, "function"
HardFault_Handler:
        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

    .weak   MemManage_Handler, "function"
MemManage_Handler:
                B       .

    .weak   BusFault_Handler, "function"
BusFault_Handler:
                B       .


    .weak   UsageFault_Handler, "function"
UsageFault_Handler:
                B       .

    .weak   SVC_Handler, "function"
SVC_Handler:
        B       .

    .weak   DebugMon_Handler, "function"
DebugMon_Handler:
                B       .

    .weak   PendSV_Handler, "function"
PendSV_Handler:
                B       .

    .weak   SysTick_Handler, "function"
SysTick_Handler:
                B       .

    .weak  Default_Handler, "function"
    .weak  BOD_IRQHandler, "function"
    .weak  IRC_IRQHandler, "function"
    .weak  PWRWU_IRQHandler, "function"
    .weak  RAMPE_IRQHandler, "function"
    .weak  CKFAIL_IRQHandler, "function"
    .weak  ISP_IRQHandler, "function"
    .weak  RTC_IRQHandler, "function"
    .weak  TAMPER_IRQHandler, "function"
    .weak  WDT_IRQHandler, "function"
    .weak  WWDT_IRQHandler, "function"
    .weak  EINT0_IRQHandler, "function"
    .weak  EINT1_IRQHandler, "function"
    .weak  EINT2_IRQHandler, "function"
    .weak  EINT3_IRQHandler, "function"
    .weak  EINT4_IRQHandler, "function"
    .weak  EINT5_IRQHandler, "function"
    .weak  GPA_IRQHandler, "function"
    .weak  GPB_IRQHandler, "function"
    .weak  GPC_IRQHandler, "function"
    .weak  GPD_IRQHandler, "function"
    .weak  GPE_IRQHandler, "function"
    .weak  GPF_IRQHandler, "function"
    .weak  QSPI0_IRQHandler, "function"
    .weak  SPI0_IRQHandler, "function"
    .weak  BRAKE0_IRQHandler, "function"
    .weak  EPWM0P0_IRQHandler, "function"
    .weak  EPWM0P1_IRQHandler, "function"
    .weak  EPWM0P2_IRQHandler, "function"
    .weak  BRAKE1_IRQHandler, "function"
    .weak  EPWM1P0_IRQHandler, "function"
    .weak  EPWM1P1_IRQHandler, "function"
    .weak  EPWM1P2_IRQHandler, "function"
    .weak  TMR0_IRQHandler, "function"
    .weak  TMR1_IRQHandler, "function"
    .weak  TMR2_IRQHandler, "function"
    .weak  TMR3_IRQHandler, "function"
    .weak  UART0_IRQHandler, "function"
    .weak  UART1_IRQHandler, "function"
    .weak  I2C0_IRQHandler, "function"
    .weak  I2C1_IRQHandler, "function"
    .weak  PDMA0_IRQHandler, "function"
    .weak  DAC_IRQHandler, "function"
    .weak  EADC00_IRQHandler, "function"
    .weak  EADC01_IRQHandler, "function"
    .weak  ACMP01_IRQHandler, "function"
    .weak  ACMP23_IRQHandler, "function"
    .weak  EADC02_IRQHandler, "function"
    .weak  EADC03_IRQHandler, "function"
    .weak  UART2_IRQHandler, "function"
    .weak  UART3_IRQHandler, "function"
    .weak  QSPI1_IRQHandler, "function"
    .weak  SPI1_IRQHandler, "function"
    .weak  SPI2_IRQHandler, "function"
    .weak  USBD_IRQHandler, "function"
    .weak  OHCI_IRQHandler, "function"
    .weak  USBOTG_IRQHandler, "function"
    .weak  BMC_IRQHandler, "function"
    .weak  SPI5_IRQHandler, "function"
    .weak  SC0_IRQHandler, "function"
    .weak  SC1_IRQHandler, "function"
    .weak  SC2_IRQHandler, "function"
    .weak  GPJ_IRQHandler, "function"
    .weak  SPI3_IRQHandler, "function"
    .weak  SPI4_IRQHandler, "function"
    .weak  SDH0_IRQHandler, "function"
    .weak  USBD20_IRQHandler, "function"
    .weak  EMAC0_IRQHandler, "function"
    .weak  I2S0_IRQHandler, "function"
    .weak  I2S1_IRQHandler, "function"
    .weak  SPI6_IRQHandler, "function"
    .weak  CRPT_IRQHandler, "function"
    .weak  GPG_IRQHandler, "function"
    .weak  EINT6_IRQHandler, "function"
    .weak  UART4_IRQHandler, "function"
    .weak  UART5_IRQHandler, "function"
    .weak  USCI0_IRQHandler, "function"
    .weak  SPI7_IRQHandler, "function"
    .weak  BPWM0_IRQHandler, "function"
    .weak  BPWM1_IRQHandler, "function"
    .weak  SPIM_IRQHandler, "function"
    .weak  CCAP_IRQHandler, "function"
    .weak  I2C2_IRQHandler, "function"
    .weak  I2C3_IRQHandler, "function"
    .weak  EQEI0_IRQHandler, "function"
    .weak  EQEI1_IRQHandler, "function"
    .weak  ECAP0_IRQHandler, "function"
    .weak  ECAP1_IRQHandler, "function"
    .weak  GPH_IRQHandler, "function"
    .weak  EINT7_IRQHandler, "function"
    .weak  SDH1_IRQHandler, "function"
    .weak  PSIO_IRQHandler, "function"
    .weak  EHCI_IRQHandler, "function"
    .weak  USBOTG20_IRQHandler, "function"
    .weak  ECAP2_IRQHandler, "function"
    .weak  ECAP3_IRQHandler, "function"
    .weak  KPI_IRQHandler, "function"
    .weak  HBI_IRQHandler, "function"
    .weak  PDMA1_IRQHandler, "function"
    .weak  UART8_IRQHandler, "function"
    .weak  UART9_IRQHandler, "function"
    .weak  TRNG_IRQHandler, "function"
    .weak  UART6_IRQHandler, "function"
    .weak  UART7_IRQHandler, "function"
    .weak  EADC10_IRQHandler, "function"
    .weak  EADC11_IRQHandler, "function"
    .weak  EADC12_IRQHandler, "function"
    .weak  EADC13_IRQHandler, "function"
    .weak  SPI8_IRQHandler, "function"
    .weak  KS_IRQHandler, "function"
    .weak  GPI_IRQHandler, "function"
    .weak  SPI9_IRQHandler, "function"
    .weak  CANFD00_IRQHandler, "function"
    .weak  CANFD01_IRQHandler, "function"
    .weak  CANFD10_IRQHandler, "function"
    .weak  CANFD11_IRQHandler, "function"
    .weak  EQEI2_IRQHandler, "function"
    .weak  EQEI3_IRQHandler, "function"
    .weak  I2C4_IRQHandler, "function"
    .weak  SPI10_IRQHandler, "function"
    .weak  CANFD20_IRQHandler, "function"
    .weak  CANFD21_IRQHandler, "function"
    .weak  CANFD30_IRQHandler, "function"
    .weak  CANFD31_IRQHandler, "function"
    .weak  EADC20_IRQHandler, "function"
    .weak  EADC21_IRQHandler, "function"
    .weak  EADC22_IRQHandler, "function"
    .weak  EADC23_IRQHandler, "function"


Default__IRQHandler:
BOD_IRQHandler:
IRC_IRQHandler:
PWRWU_IRQHandler:
RAMPE_IRQHandler:
CKFAIL_IRQHandler:
ISP_IRQHandler:
RTC_IRQHandler:
TAMPER_IRQHandler:
WDT_IRQHandler:
WWDT_IRQHandler:
EINT0_IRQHandler:
EINT1_IRQHandler:
EINT2_IRQHandler:
EINT3_IRQHandler:
EINT4_IRQHandler:
EINT5_IRQHandler:
GPA_IRQHandler:
GPB_IRQHandler:
GPC_IRQHandler:
GPD_IRQHandler:
GPE_IRQHandler:
GPF_IRQHandler:
QSPI0_IRQHandler:
SPI0_IRQHandler:
BRAKE0_IRQHandler:
EPWM0P0_IRQHandler:
EPWM0P1_IRQHandler:
EPWM0P2_IRQHandler:
BRAKE1_IRQHandler:
EPWM1P0_IRQHandler:
EPWM1P1_IRQHandler:
EPWM1P2_IRQHandler:
TMR0_IRQHandler:
TMR1_IRQHandler:
TMR2_IRQHandler:
TMR3_IRQHandler:
UART0_IRQHandler:
UART1_IRQHandler:
I2C0_IRQHandler:
I2C1_IRQHandler:
PDMA0_IRQHandler:
DAC_IRQHandler:
EADC00_IRQHandler:
EADC01_IRQHandler:
ACMP01_IRQHandler:
ACMP23_IRQHandler:
EADC02_IRQHandler:
EADC03_IRQHandler:
UART2_IRQHandler:
UART3_IRQHandler:
QSPI1_IRQHandler:
SPI1_IRQHandler:
SPI2_IRQHandler:
USBD_IRQHandler:
OHCI_IRQHandler:
USBOTG_IRQHandler:
BMC_IRQHandler:
SPI5_IRQHandler:
SC0_IRQHandler:
SC1_IRQHandler:
SC2_IRQHandler:
GPJ_IRQHandler:
SPI3_IRQHandler:
SPI4_IRQHandler:
SDH0_IRQHandler:
USBD20_IRQHandler:
EMAC0_IRQHandler:
I2S0_IRQHandler:
I2S1_IRQHandler:
SPI6_IRQHandler:
CRPT_IRQHandler:
GPG_IRQHandler:
EINT6_IRQHandler:
UART4_IRQHandler:
UART5_IRQHandler:
USCI0_IRQHandler:
SPI7_IRQHandler:
BPWM0_IRQHandler:
BPWM1_IRQHandler:
SPIM_IRQHandler:
CCAP_IRQHandler:
I2C2_IRQHandler:
I2C3_IRQHandler:
EQEI0_IRQHandler:
EQEI1_IRQHandler:
ECAP0_IRQHandler:
ECAP1_IRQHandler:
GPH_IRQHandler:
EINT7_IRQHandler:
SDH1_IRQHandler:
PSIO_IRQHandler:
EHCI_IRQHandler:
USBOTG20_IRQHandler:
ECAP2_IRQHandler:
ECAP3_IRQHandler:
KPI_IRQHandler:
HBI_IRQHandler:
PDMA1_IRQHandler:
UART8_IRQHandler:
UART9_IRQHandler:
TRNG_IRQHandler:
UART6_IRQHandler:
UART7_IRQHandler:
EADC10_IRQHandler:
EADC11_IRQHandler:
EADC12_IRQHandler:
EADC13_IRQHandler:
SPI8_IRQHandler:
KS_IRQHandler:
GPI_IRQHandler:
SPI9_IRQHandler:
CANFD00_IRQHandler:
CANFD01_IRQHandler:
CANFD10_IRQHandler:
CANFD11_IRQHandler:
EQEI2_IRQHandler:
EQEI3_IRQHandler:
I2C4_IRQHandler:
SPI10_IRQHandler:
CANFD20_IRQHandler:
CANFD21_IRQHandler:
CANFD30_IRQHandler:
CANFD31_IRQHandler:
EADC20_IRQHandler:
EADC21_IRQHandler:
EADC22_IRQHandler:
EADC23_IRQHandler:
        B       .
