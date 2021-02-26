/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate TIMER PWM accumulator interrupt to stop counting.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t gu32Period;


void TMR0_IRQHandler(void)
{
    TPWM_ClearAccInt(TIMER0);
    
    printf("\nCheck if output toggles 11 times then stop toggles.\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));  
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    
    /* Set Timer0 PWM CH0(TM0) pin */
    SET_TM0_PB5();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------------+\n");
    printf("|    Timer PWM Accumulator Inerrupt Stop Mode Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Timer0 PWM accumulator stop mode.\n");
    printf("  When accumulator interrupt happens, the output of Timer0 PWM stops.\n");
    printf("  Since interrupt accumulator count is set to 10, the output toggles 11 times then stops.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");

    printf("\n\nPress any key to start Timer0 PWM.\n");
    getchar();

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Set PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_DOWN_COUNT);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable Timer0 PWM accumulator function, interrupt count 10, accumulator source select to zero point */
    TPWM_EnableAcc(TIMER0, 10, TPWM_IFA_ZERO_POINT);

    /* Enable Timer0 PWM accumulator interrupt */
    TPWM_EnableAccInt(TIMER0);

    /* Enable Timer0 PWM accumulator stop mode */
    TPWM_EnableAccStopMode(TIMER0);

    /* Enable Timer0 interrupt */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);

    while(1) {}
}
