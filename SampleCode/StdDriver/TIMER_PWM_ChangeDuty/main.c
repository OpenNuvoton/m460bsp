/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Change duty cycle and period of output waveform in PWM down count type.
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
    static uint32_t u32Toggle = 0;

    if(TPWM_GET_PERIOD_INT_FLAG(TIMER0))
    {
        if(u32Toggle == 0)
        {
            /* Set PWM period to generate output frequency 36000 Hz */
            TPWM_SET_PERIOD(TIMER0, ((gu32Period/2)-1));

            /* Set PWM duty, 40% */
            TPWM_SET_CMPDAT(TIMER0, (((gu32Period/2)*4) / 10));
        }
        else
        {
            /* Set PWM period to generate output frequency 18000 Hz */
            TPWM_SET_PERIOD(TIMER0, (gu32Period-1));

            /* Set PWM duty, 50% */
            TPWM_SET_CMPDAT(TIMER0, (gu32Period/2));
        }
        u32Toggle ^= 1;
        TPWM_CLEAR_PERIOD_INT_FLAG(TIMER0);
    }
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

    printf("+-----------------------------------------------+\n");
    printf("|    Timer PWM Change Duty Cycle Sample Code    |\n");
    printf("+-----------------------------------------------+\n\n");

    printf("# Timer0 PWM_CH0 frequency of first period is 18000 Hz and duty is 50%%.\n");
    printf("# Timer0 PWM_CH0 frequency of second period is 36000 Hz and duty is 40%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Get initial period and comparator value */
    gu32Period = TPWM_GET_PERIOD(TIMER0) + 1;

    /* Set PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_DOWN_COUNT);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable period event interrupt */
    TPWM_ENABLE_PERIOD_INT(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);

    printf("*** Check Timer0 PWM_CH0 output waveform by oscilloscope ***\n");

    while(1) {}
}
