/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate Timer PWM Complementary mode and Dead-Time function.
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

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    
    /* Set Timer0 PWM CH0, CH1 and Timer1 PWM CH0, CH1 */
    SET_TM0_PB5();
    SET_TM0_EXT_PH0();
    SET_TM1_PB4();
    SET_TM1_EXT_PH1();
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
    uint32_t u32Period, u32CMP, u32Prescaler, u32DeadTime;
    
    /* Unlock protected registers */
    SYS_UnlockReg();
    
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("+--------------------------------------------------------------+\n");
    printf("|    Timer PWM Complementary mode and Dead-Time Sample Code    |\n");
    printf("+--------------------------------------------------------------+\n\n");

    /* Configure Timer0 PWM */
    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as complementary mode*/
    TPWM_ENABLE_COMPLEMENTARY_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 6000 Hz, duty 40% */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 6000, 40);

    /* Enable output of PWM_CH0 and PWM_CH1 */
    TPWM_ENABLE_OUTPUT(TIMER0, (TPWM_CH1|TPWM_CH0));

    /* Get u32Prescaler, u32Period and u32CMP after called TPWM_ConfigOutputFreqAndDuty() API */
    u32Prescaler = (TIMER0->PWMCLKPSC + 1);
    u32Period = (TIMER0->PWMPERIOD + 1);
    u32CMP = TIMER0->PWMCMPDAT;
    u32DeadTime = u32CMP/2;

    printf("# Timer0 PWM output frequency is 600 Hz and duty 40%%.\n");
    printf("    - Counter clock source:    PCLK0 \n");
    printf("    - Counter clock prescaler: %d \n", u32Prescaler);
    printf("    - Counter type:            Up count type \n");
    printf("    - Operation mode:          Complementary in auto-reload mode \n");
    printf("    - Period value:            %d \n", u32Period);
    printf("    - Comparator value:        %d \n", u32CMP);
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PB.5, PWM_CH1 on PH.0\n\n");

    /* Configure Timer1 PWM */
    printf("# Timer1 PWM output frequency is 600 Hz and duty 20%% with dead-time insertion.\n");
    printf("    - Counter clock source:    PCLK0 \n");
    printf("    - Counter clock prescaler: %d \n", u32Prescaler);
    printf("    - Counter type:            Up count type \n");
    printf("    - Operation mode:          Complementary in auto-reload mode \n");
    printf("    - Period value:            %d \n", u32Period);
    printf("    - Comparator value:        %d \n", u32CMP);
    printf("    - Dead-Time interval:      %d \n", u32DeadTime);
    printf("# I/O configuration:\n");
    printf("    - Timer1 PWM_CH0 on PB.4, PWM_CH1 on PH.1\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER1);

    /* Set PWM mode as complementary mode*/
    TPWM_ENABLE_COMPLEMENTARY_MODE(TIMER1);

    /* Set Timer1 PWM output frequency is 6000 Hz, duty 40% */
    TPWM_ConfigOutputFreqAndDuty(TIMER1, 6000, 40);

    /* Enable output of PWM_CH0 and PWM_CH1 */
    TPWM_ENABLE_OUTPUT(TIMER1, (TPWM_CH1|TPWM_CH0));

    /* Enable and configure dead-time interval is (u32DeadTime * TMR1_PWMCLK * prescaler) */
    SYS_UnlockReg(); // Unlock protected registers
    TPWM_EnableDeadTimeWithPrescale(TIMER1, (u32DeadTime-1));
    SYS_LockReg(); // Lock protected registers

    printf("*** Check Timer0 and Timer1 PWM output waveform by oscilloscope ***\n");

    /* Start Timer0 and Timer1 PWM counter by trigger Timer0 sync. start */
    TPWM_SET_COUNTER_SYNC_MODE(TIMER0, TPWM_CNTR_SYNC_START_BY_TIMER0);
    TPWM_SET_COUNTER_SYNC_MODE(TIMER1, TPWM_CNTR_SYNC_START_BY_TIMER0);
    TPWM_TRIGGER_COUNTER_SYNC(TIMER0);

    while(1) {}
}
