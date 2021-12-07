;/******************************************************************************
; * @file     startup_m460_user.s
; * @version  V3.00
; * @brief    CMSIS Cortex-M4 Core Device Startup File for M460
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00001000
	ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000100
	ENDIF

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                ; Remove all External Interrupts to reduce code size
                ; This interrupts table is not used in polling mode


__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

    
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
 
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                IMPORT  ProcessHardFault
                EXPORT  HardFault_Handler         [WEAK]
                ;B       .
                MOV     R0, LR
                MRS     R1, MSP
                MRS     R2, PSP
                LDR     R3, =ProcessHardFault
                BLX     R3
                BX      R0
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  IRC_IRQHandler            [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  RAMPE_IRQHandler          [WEAK]
                EXPORT  CKFAIL_IRQHandler         [WEAK]
                EXPORT  ISP_IRQHandler            [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  TAMPER_IRQHandler         [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  WWDT_IRQHandler           [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  EINT2_IRQHandler          [WEAK]
                EXPORT  EINT3_IRQHandler          [WEAK]
                EXPORT  EINT4_IRQHandler          [WEAK]
                EXPORT  EINT5_IRQHandler          [WEAK]
                EXPORT  GPA_IRQHandler            [WEAK]
                EXPORT  GPB_IRQHandler            [WEAK]
                EXPORT  GPC_IRQHandler            [WEAK]
                EXPORT  GPD_IRQHandler            [WEAK]
                EXPORT  GPE_IRQHandler            [WEAK]
                EXPORT  GPF_IRQHandler            [WEAK]
                EXPORT  QSPI0_IRQHandler          [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  BRAKE0_IRQHandler         [WEAK]
                EXPORT  EPWM0P0_IRQHandler        [WEAK]
                EXPORT  EPWM0P1_IRQHandler        [WEAK]
                EXPORT  EPWM0P2_IRQHandler        [WEAK]
                EXPORT  BRAKE1_IRQHandler         [WEAK]
                EXPORT  EPWM1P0_IRQHandler        [WEAK]
                EXPORT  EPWM1P1_IRQHandler        [WEAK]
                EXPORT  EPWM1P2_IRQHandler        [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  PDMA0_IRQHandler          [WEAK]
                EXPORT  DAC_IRQHandler            [WEAK]
                EXPORT  EADC00_IRQHandler         [WEAK]
                EXPORT  EADC01_IRQHandler         [WEAK]
                EXPORT  ACMP01_IRQHandler         [WEAK]
                EXPORT  ACMP23_IRQHandler         [WEAK]
                EXPORT  EADC02_IRQHandler         [WEAK]
                EXPORT  EADC03_IRQHandler         [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  QSPI1_IRQHandler          [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  SPI2_IRQHandler           [WEAK]
                EXPORT  USBD_IRQHandler           [WEAK]
                EXPORT  OHCI_IRQHandler           [WEAK]
                EXPORT  USBOTG_IRQHandler         [WEAK]
                EXPORT  SPI5_IRQHandler           [WEAK]
                EXPORT  SC0_IRQHandler            [WEAK]
                EXPORT  SC1_IRQHandler            [WEAK]
                EXPORT  SC2_IRQHandler            [WEAK]
                EXPORT  GPJ_IRQHandler            [WEAK]
                EXPORT  SPI3_IRQHandler           [WEAK]
                EXPORT  SPI4_IRQHandler           [WEAK]
                EXPORT  SDH0_IRQHandler           [WEAK]
                EXPORT  USBD20_IRQHandler         [WEAK]
                EXPORT  EMAC0_IRQHandler          [WEAK]
                EXPORT  I2S0_IRQHandler           [WEAK]
                EXPORT  I2S1_IRQHandler           [WEAK]
                EXPORT  SPI6_IRQHandler           [WEAK]
                EXPORT  CRPT_IRQHandler           [WEAK]
                EXPORT  GPG_IRQHandler            [WEAK]
                EXPORT  EINT6_IRQHandler          [WEAK]
                EXPORT  UART4_IRQHandler          [WEAK]
                EXPORT  UART5_IRQHandler          [WEAK]
                EXPORT  USCI0_IRQHandler          [WEAK]
                EXPORT  SPI7_IRQHandler           [WEAK]
                EXPORT  BPWM0_IRQHandler          [WEAK]
                EXPORT  BPWM1_IRQHandler          [WEAK]
                EXPORT  SPIM_IRQHandler           [WEAK]
                EXPORT  CCAP_IRQHandler           [WEAK]
                EXPORT  I2C2_IRQHandler           [WEAK]
                EXPORT  I2C3_IRQHandler           [WEAK]
                EXPORT  EQEI0_IRQHandler          [WEAK]
                EXPORT  EQEI1_IRQHandler          [WEAK]
                EXPORT  ECAP0_IRQHandler          [WEAK]
                EXPORT  ECAP1_IRQHandler          [WEAK]
                EXPORT  GPH_IRQHandler            [WEAK]
                EXPORT  EINT7_IRQHandler          [WEAK]
                EXPORT  SDH1_IRQHandler           [WEAK]
                EXPORT  PSIO_IRQHandler           [WEAK]
                EXPORT  EHCI_IRQHandler           [WEAK]
                EXPORT  USBOTG20_IRQHandler       [WEAK]
                EXPORT  ECAP2_IRQHandler          [WEAK]
                EXPORT  ECAP3_IRQHandler          [WEAK]
                EXPORT  KPI_IRQHandler            [WEAK]
                EXPORT  HBI_IRQHandler            [WEAK]
                EXPORT  PDMA1_IRQHandler          [WEAK]
                EXPORT  UART8_IRQHandler          [WEAK]
                EXPORT  UART9_IRQHandler          [WEAK]
                EXPORT  TRNG_IRQHandler           [WEAK]
                EXPORT  UART6_IRQHandler          [WEAK]
                EXPORT  UART7_IRQHandler          [WEAK]
                EXPORT  EADC10_IRQHandler         [WEAK]
                EXPORT  EADC11_IRQHandler         [WEAK]
                EXPORT  EADC12_IRQHandler         [WEAK]
                EXPORT  EADC13_IRQHandler         [WEAK]
                EXPORT  SPI8_IRQHandler           [WEAK]
                EXPORT  KS_IRQHandler             [WEAK]
                EXPORT  GPI_IRQHandler            [WEAK]
                EXPORT  SPI9_IRQHandler           [WEAK]
                EXPORT  CANFD00_IRQHandler        [WEAK]
                EXPORT  CANFD01_IRQHandler        [WEAK]
                EXPORT  CANFD10_IRQHandler        [WEAK]
                EXPORT  CANFD11_IRQHandler        [WEAK]
                EXPORT  EQEI2_IRQHandler          [WEAK]
                EXPORT  EQEI3_IRQHandler          [WEAK]
                EXPORT  I2C4_IRQHandler           [WEAK]
                EXPORT  SPI10_IRQHandler          [WEAK]
                EXPORT  CANFD20_IRQHandler        [WEAK]
                EXPORT  CANFD21_IRQHandler        [WEAK]
                EXPORT  CANFD30_IRQHandler        [WEAK]
                EXPORT  CANFD31_IRQHandler        [WEAK]
                EXPORT  EADC20_IRQHandler         [WEAK]
                EXPORT  EADC21_IRQHandler         [WEAK]
                EXPORT  EADC22_IRQHandler         [WEAK]
                EXPORT  EADC23_IRQHandler         [WEAK]


Default__IRQHandler
BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
RAMPE_IRQHandler
CKFAIL_IRQHandler
ISP_IRQHandler
RTC_IRQHandler
TAMPER_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT4_IRQHandler
EINT5_IRQHandler
GPA_IRQHandler
GPB_IRQHandler
GPC_IRQHandler
GPD_IRQHandler
GPE_IRQHandler
GPF_IRQHandler
QSPI0_IRQHandler
SPI0_IRQHandler
BRAKE0_IRQHandler
EPWM0P0_IRQHandler
EPWM0P1_IRQHandler
EPWM0P2_IRQHandler
BRAKE1_IRQHandler
EPWM1P0_IRQHandler
EPWM1P1_IRQHandler
EPWM1P2_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
PDMA0_IRQHandler
DAC_IRQHandler
EADC00_IRQHandler
EADC01_IRQHandler
ACMP01_IRQHandler
ACMP23_IRQHandler
EADC02_IRQHandler
EADC03_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
QSPI1_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USBD_IRQHandler
OHCI_IRQHandler
USBOTG_IRQHandler
SPI5_IRQHandler
SC0_IRQHandler
SC1_IRQHandler
SC2_IRQHandler
GPJ_IRQHandler
SPI3_IRQHandler
SPI4_IRQHandler
SDH0_IRQHandler
USBD20_IRQHandler
EMAC0_IRQHandler
I2S0_IRQHandler
I2S1_IRQHandler
SPI6_IRQHandler
CRPT_IRQHandler
GPG_IRQHandler
EINT6_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
USCI0_IRQHandler
SPI7_IRQHandler
BPWM0_IRQHandler
BPWM1_IRQHandler
SPIM_IRQHandler
CCAP_IRQHandler
I2C2_IRQHandler
I2C3_IRQHandler
EQEI0_IRQHandler
EQEI1_IRQHandler
ECAP0_IRQHandler
ECAP1_IRQHandler
GPH_IRQHandler
EINT7_IRQHandler
SDH1_IRQHandler
PSIO_IRQHandler
EHCI_IRQHandler
USBOTG20_IRQHandler
ECAP2_IRQHandler
ECAP3_IRQHandler
KPI_IRQHandler
HBI_IRQHandler
PDMA1_IRQHandler
UART8_IRQHandler
UART9_IRQHandler
TRNG_IRQHandler
UART6_IRQHandler
UART7_IRQHandler
EADC10_IRQHandler
EADC11_IRQHandler
EADC12_IRQHandler
EADC13_IRQHandler
SPI8_IRQHandler
KS_IRQHandler
GPI_IRQHandler
SPI9_IRQHandler
CANFD00_IRQHandler
CANFD01_IRQHandler
CANFD10_IRQHandler
CANFD11_IRQHandler
EQEI2_IRQHandler
EQEI3_IRQHandler
I2C4_IRQHandler
SPI10_IRQHandler
CANFD20_IRQHandler
CANFD21_IRQHandler
CANFD30_IRQHandler
CANFD31_IRQHandler
EADC20_IRQHandler
EADC21_IRQHandler
EADC22_IRQHandler
EADC23_IRQHandler


                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
