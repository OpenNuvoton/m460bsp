/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate output different duty waveform in Timer0~Timer3 PWM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART_Init(void);


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
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
    
    /* Set Timer0~4 PWM output pins */
    SET_TM0_PG2();
    SET_TM1_PG3();
    SET_TM2_PG4();
    SET_TM3_PF11();
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
    printf("+-------------------------------------------------+\n");
    printf("|    Timer0~Timer3 PWM Output Duty Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer0 PWM_CH0 output frequency is 18000 Hz and duty is 50%%.\n");
    printf("# Timer1 PWM_CH0 output frequency is 10000 Hz and duty is 10%%.\n");
    printf("# Timer2 PWM_CH0 output frequency is  9000 Hz and duty is 75%%.\n");
    printf("# Timer3 PWM_CH0 output frequency is  4000 Hz and duty is 20%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PG.2\n");
    printf("    - Timer1 PWM_CH0 on PG.3\n");
    printf("    - Timer2 PWM_CH0 on PG.4\n");
    printf("    - Timer3 PWM_CH0 on PF.11\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);
    TPWM_ENABLE_PWM_MODE(TIMER1);
    TPWM_ENABLE_PWM_MODE(TIMER2);
    TPWM_ENABLE_PWM_MODE(TIMER3);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER1);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER2);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER3);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER1, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER2, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Set Timer1 PWM output frequency is 10000 Hz, duty 10% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER1, 10000, 10);

    /* Set Timer2 PWM output frequency is 9000 Hz, duty 75% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER2, 9000, 75);

    /* Set Timer3 PWM output frequency is 4000 Hz, duty 20% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER3, 4000, 20);

    /* Set PWM up count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER1, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER2, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER3, TPWM_UP_COUNT);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);
    TPWM_START_COUNTER(TIMER1);
    TPWM_START_COUNTER(TIMER2);
    TPWM_START_COUNTER(TIMER3);

    printf("*** Check Timer0~Timer3 PWM_CH0 output waveform by oscilloscope ***\n");

    while(1) {}
}
