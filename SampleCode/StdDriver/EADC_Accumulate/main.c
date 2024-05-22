/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to get accumulate conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


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
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* @brief Left shift Accumulate raw data back to correct conversion result                                 */
/* @param[in] accu_raw_result  Raw data of Accumulate conversion                                           */
/* @param[in] accu_count       Conversion count of Accumulate conversion.                                  */
/*                             Valid values are 1, 2, 4, 8, 16, 32, 64, 128, 256.                          */
/* @return The correct conversion result of Accumulate conversion                                          */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t EADC_Accumulate_LeftShift(uint32_t u32AccuRawResult, uint32_t u32AccuCount)
{
    switch(u32AccuCount)
    {
        case 1:
            return u32AccuRawResult;
        case 2:
            return u32AccuRawResult;
        case 4:
            return u32AccuRawResult;
        case 8:
            return u32AccuRawResult;
        case 16:
            return u32AccuRawResult;
        case 32:
            return (u32AccuRawResult << 1);
        case 64:
            return (u32AccuRawResult << 2);
        case 128:
            return (u32AccuRawResult << 3);
        case 256:
            return (u32AccuRawResult << 4);
        default:
            printf("*** Error! Wrong parameter %u for Accumulate.\n\n", u32AccuCount);
            return u32AccuRawResult;
    }
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData;
    uint32_t u32TimeOutCnt;
    int32_t  i32Err;

    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|            EADC Accumulate sample code            |\n");
    printf("+---------------------------------------------------+\n");

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

    while(1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC conversion (channel 0 only)\n");
        printf("  [2] Basic EADC conversion (channel 1 only)\n");
        printf("  Other keys: exit EADC test\n");
        u8Option = getchar();

        if(u8Option == '1')
            u32ChannelNum = 0;
        else if(u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC0, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 0 */
        EADC_SetExtendSampleTime(EADC0, u32ModuleNum, 0);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC0, u32ModuleNum, EADC_MCTL1_ACU_32);

        /* Disable Average feature */
        EADC_DISABLE_AVG(EADC0, u32ModuleNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC00_IRQn);

        /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32AdcIntFlag = 0;
        EADC_START_CONV(EADC0, u32ModuleMask);

        /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(g_u32AdcIntFlag == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EADC interrupt time-out!\n");
                return;
            }
        }

        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC0, u32ModuleNum);
        printf("Conversion result of channel %u accumulated 32 times : 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        i32ConversionData = EADC_Accumulate_LeftShift(i32ConversionData, 32);
        printf("Conversion result of channel %u after left shifted   : 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        printf("The average that calculated by software is            0x%X (%d)\n\n", (int)(i32ConversionData / 32), (int)(i32ConversionData / 32));

        /* Disable Accumulate feature */
        EADC_DISABLE_ACU(EADC0, u32ModuleNum);

        /* Disable the sample module interrupt. */
        EADC_DISABLE_INT(EADC0, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC00_IRQn);
    }   /* end of while(1) */
}

void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    printf("Exit EADC sample code\n");

    while(1);
}
