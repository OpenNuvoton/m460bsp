/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use BPWM0 channel 0 to capture the BPWM1 channel 0 waveform.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);
int32_t CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch);
void UART0_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* au32Count[4] : Keep the internal counter value when input signal rising / falling    */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
int32_t CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch)
{
    uint16_t au32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

    /* Wait for Capture Falling Indicator */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        au32Count[u32i++] = (uint16_t)BPWM_GET_CAPTURE_FALLING_DATA(BPWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        au32Count[u32i++] = (uint16_t)BPWM_GET_CAPTURE_RISING_DATA(BPWM, u32Ch);
    }

    u16RisingTime = au32Count[1];

    u16FallingTime = au32Count[0];

    u16HighPeriod = au32Count[1] - au32Count[2];

    u16LowPeriod = (uint16_t)(0x10000 - (uint32_t)au32Count[1]);

    u16TotalPeriod = (uint16_t)(0x10000 - (uint32_t)au32Count[2]);

    printf("\nBPWM generate: \nHigh Period=17141 ~ 17143, Low Period=39999 ~ 40001, Total Period=57141 ~ 57143\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 17141) || (u16HighPeriod > 17143) || (u16LowPeriod < 39999) || (u16LowPeriod > 40001) || (u16TotalPeriod < 57141) || (u16TotalPeriod > 57143))
    {
        printf("Capture Test Fail!!\n");
        return -1;
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
}

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

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

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);

    /* Select BPWM module clock source */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(BPWM1_MODULE, CLK_CLKSEL2_BPWM1SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for BPWM */
    SET_BPWM0_CH0_PE2();
    SET_BPWM1_CH0_PB11();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from BPWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(PE.2 BPWM0 channel 0) <--> BPWM1_CH0(PB.11 BPWM1 channel 0)\n\n");
    printf("Use BPWM0 Channel 0(PE.2) to capture the BPWM1 Channel 0(PB.11) Waveform\n");

    while(1)
    {
        printf("Press any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM1 Channel 0 as BPWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume BPWM output frequency is 250Hz and duty ratio is 30%, user can calculate BPWM settings by follows.(up counter type)
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           BPWM clock source frequency = PLL/2 = 100000000
           (CNR+1) = PWM clock source frequency/prescaler/BPWM output frequency
                   = 100000000/7/250 = 57142
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 57141
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = 17142
           Prescale value is 6 : prescaler= 7
        */

        /* Set BPWM1 channel 0 output configuration */
        BPWM_ConfigOutputChannel(BPWM1, 0, 250, 30);

        /* Enable BPWM Output path for BPWM1 channel 0 */
        BPWM_EnableOutput(BPWM1, BPWM_CH_0_MASK);

        /* Enable Timer for BPWM1 channel 0 */
        BPWM_Start(BPWM1, BPWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 100000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 100000000/7/250 = 57142
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1/Capture clock source frequency/prescaler
           70 ns = 1/100000000/7
        */

        /* Set BPWM0 channel 0 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, 0, 70, 0);

        /* Enable Timer for BPWM0 channel 0 */
        BPWM_Start(BPWM0, BPWM_CH_0_MASK);

        /* Enable Capture Function for BPWM0 channel 0 */
        BPWM_EnableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* Wait until BPWM0 channel 0 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((BPWM0->CNT) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 Timer start to count time-out!\n");
                goto lexit;
            }
        }

        /* Capture the Input Waveform Data */
        if(CalPeriodTime(BPWM0, 0) < 0)
            goto lexit;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM1 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set BPWM1 channel 0 loaded value as 0 */
        BPWM_Stop(BPWM1, BPWM_CH_0_MASK);

        /* Wait until BPWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((BPWM1->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 Timer Stop time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for BPWM1 channel 0 */
        BPWM_ForceStop(BPWM1, BPWM_CH_0_MASK);

        /* Disable BPWM Output path for BPWM1 channel 0 */
        BPWM_DisableOutput(BPWM1, BPWM_CH_0_MASK);

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM_Stop(BPWM0, BPWM_CH_0_MASK);

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock;  /* 1 second time-out */
        while((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 current counter reach to 0 time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM_DisableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }

lexit:

    while(1);
}
