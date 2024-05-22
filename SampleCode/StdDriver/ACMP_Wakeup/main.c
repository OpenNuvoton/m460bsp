/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up MCU from Power-down mode by ACMP wake-up function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define WAIT_UART()     do{\
                            int timeout = 100000;\
                            if(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)\
                                break;\
                            if(timeout-- < 0)\
                                break;\
                        }while(1)

/* Function prototype declaration */
void SYS_Init(void);
void PowerDownFunction(void);
int IsDebugFifoEmpty(void);
void ACMP01_IRQHandler(void);


int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    //SYS_LockReg();

    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(DEBUG_PORT, 115200);

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|             ACMP Sample Code          |\n");
    printf("+---------------------------------------+\n");

    printf("CPU @ %d\n", SystemCoreClock);

    printf("\nThis sample code demonstrates ACMP1 wake-up function. Using ACMP1_P1 (PB4) as ACMP\n");
    printf("positive input and using internal CRV as the negative input.\n");
    printf("The compare result reflects on ACMP1_O (PB6).\n");

    printf("When the voltage of the positive input is greater than the voltage of the negative input,\n");
    printf("the analog comparator outputs logical one; otherwise, it outputs logical zero.\n");
    printf("This chip will be waked up from power down mode when detecting a transition of analog comparator's output.\n");
    printf("Clock is output to PB14 to monitor the pwoer down.\n");
    printf("ICE connection will cause fail to power down. A power cycle is necessary after ICE connected.\n");
    printf("\n");


    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV1_SRC(ACMP01, ACMP_VREF_CRV1SSEL_VDDA);

    /* Select CRV level */
    ACMP_CRV1_SEL(ACMP01, 16);

    /* Enable CRV1 */
    ACMP_ENABLE_CRV1(ACMP01);

    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_10MV);
    ACMP_SET_FILTER(ACMP01, 1, ACMP_CTL_FILTSEL_16PCLK);

    /* Use P1 as input */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 1);

    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Init CKO to monitor system clock */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 15, 0);
    SET_CLKO_PB14();

    for(;;)
    {
        printf("Enter power-down ....\n");
        /* Wait message print out */
        WAIT_UART();

        PowerDownFunction();
        printf("Wake up by ACMP0!\n");

        CLK_SysTickLongDelay(3000000);

    }

}

void ACMP01_IRQHandler(void)
{
    static int cnt = 0;
    printf("\nACMP interrupt! %d\n", cnt++);
    /* Disable interrupt */
    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Clear wake-up interrupt flag */
    ACMP_CLR_WAKEUP_INT_FLAG(ACMP01, 1);
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pin for ACMP1 positive input pin */
    SET_ACMP1_P1_PB4();

    /* Set multi-function pin for ACMP1 output pin */
    SET_ACMP1_O_PB6();

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Disable digital input path of analog pin ACMP0_P0 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);

}


void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    printf("\nSystem enter power-down mode ... ");

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Deep sleep mode is selected */
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;

    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);

    __WFI();
}
