/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32AdcCmp0IntFlag;
static volatile uint32_t g_u32AdcCmp1IntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void EADC03_IRQHandler(void);


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

    /* Enable EADC0 module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Set EADC0 clock divider as 12 */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_PLL_DIV2, CLK_CLKDIV0_EADC0(12));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for EADC0 channels. */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();
    SET_EADC0_CH2_PB2();
    SET_EADC0_CH3_PB3();

    /* Disable digital input path of EADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT3 | BIT2 | BIT1 | BIT0);

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
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    uint32_t u32TimeOutCnt;
    int32_t  i32Err;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|           EADC compare function (result monitor) sample code         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Set input mode as single-end and enable the A/D converter */
    i32Err = EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    /* Check EADC global error code. */
    if(i32Err != 0)
    {
        if(i32Err == EADC_CAL_ERR)
        {
            printf("EADC has calibration error.\n");
            return;
        }
        else if(i32Err == EADC_CLKDIV_ERR)
        {
            printf("EADC clock frequency is configured error.\n");
            return;
        }
        else
        {
            printf("EADC has operation error.\n");
            return;
        }
    }

    /* Configure the sample module 0 for analog input channel 2 and ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC0, 0, EADC_ADINT0_TRIGGER, 2);

    /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    EADC_ENABLE_CMP0(EADC0, 0, EADC_CMP_CMPCOND_LESS_THAN, 0x800, 5);

    /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1 : channel 2 is greater than or equal to 0x800; match count is 5.\n");
    EADC_ENABLE_CMP1(EADC0, 0, EADC_CMP_CMPCOND_GREATER_OR_EQUAL, 0x800, 5);

    /* Enable sample module 0 for ADINT0 */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);
    /* Enable ADINT0 interrupt */
    EADC_ENABLE_INT(EADC0, BIT0);

    /* Clear the A/D ADINT3 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF3_Msk);
    /* Enable sample module 0 for ADINT3 */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 3, BIT0);
    /* Enable ADINT3 interrupt */
    EADC_ENABLE_INT(EADC0, BIT3);
    NVIC_EnableIRQ(EADC03_IRQn);

    /* Clear the EADC comparator 0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF0_Msk);
    /* Enable ADC comparator 0 interrupt */
    EADC_ENABLE_CMP_INT(EADC0, 0);

    /* Clear the EADC comparator 1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF1_Msk);
    /* Enable ADC comparator 1 interrupt */
    EADC_ENABLE_CMP_INT(EADC0, 1);

    /* Reset the EADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    EADC_START_CONV(EADC0, BIT0);

    /* Wait EADC compare interrupt */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((g_u32AdcCmp0IntFlag == 0) && (g_u32AdcCmp1IntFlag == 0))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC compare interrupt time-out!\n");
            return;
        }
    }

    /* Disable the sample module 0 interrupt */
    EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);

    /* Disable ADC comparator interrupt */
    EADC_DISABLE_CMP_INT(EADC0, 0);
    EADC_DISABLE_CMP_INT(EADC0, 1);
    /* Disable compare function */
    EADC_DISABLE_CMP0(EADC0);
    EADC_DISABLE_CMP1(EADC0);

    if(g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC03_IRQHandler(void)
{
    if(EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF0_Msk))
    {
        g_u32AdcCmp0IntFlag = 1;
        /* Clear the A/D compare flag 0 */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF0_Msk);
    }

    if(EADC_GET_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF1_Msk))
    {
        g_u32AdcCmp1IntFlag = 1;
        /* Clear the A/D compare flag 1 */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADCMPF1_Msk);
    }
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF3_Msk);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC0_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC03_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
