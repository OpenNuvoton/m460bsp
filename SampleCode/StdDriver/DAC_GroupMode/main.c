/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate DAC0 and DAC1 work in group mode
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

static const uint16_t g_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                                      4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                                      3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                                      639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                                      238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                                     };

static const uint32_t g_u32ArraySize = sizeof(g_au16Sine) / sizeof(uint16_t);
static uint32_t g_u32Index = 0;
static uint32_t g_u32Dac0Done = 0, g_u32Dac1Done = 0;

void DAC_IRQHandler(void);
void SYS_Init(void);



void DAC_IRQHandler(void)
{
    if(DAC_GET_INT_FLAG(DAC0, 0))
    {
        /* Clear the DAC conversion complete finish flag */
        DAC_CLR_INT_FLAG(DAC0, 0);
        DAC_WRITE_DATA(DAC0, 0, g_au16Sine[g_u32Index]);
        g_u32Dac0Done = 1;

    }
    if(DAC_GET_INT_FLAG(DAC1, 0))
    {

        /* Clear the DAC conversion complete finish flag */
        DAC_CLR_INT_FLAG(DAC1, 0);
        DAC_WRITE_DATA(DAC1, 0, g_au16Sine[g_u32Index >= g_u32ArraySize / 2 ? g_u32Index - g_u32ArraySize / 2 : g_u32Index + g_u32ArraySize / 2]);
        g_u32Dac1Done = 1;

        if(++g_u32Index == g_u32ArraySize)
            g_u32Index = 0;
    }

    if(g_u32Dac0Done == 1 && g_u32Dac1Done == 1)
    {
        DAC_START_CONV(DAC0);
        g_u32Dac0Done = g_u32Dac1Done = 0;
    }

    return;
}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    //SET_UART0_RXD_PB12();//conflict with DAC0_OUT pin
    SET_UART0_TXD_PB13();//conflict with DAC1_OUT pin

}

int main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          DAC Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("DAC0 and DAC1 is configured in group mode and update simultaneously\n");

    /* Set multi-function pin for DAC voltage output */
    SET_DAC0_OUT_PB12();//conflict with UART0_RXD pin
    SET_DAC1_OUT_PB13();//conflict with UART0_TXD pin

    /* Disable digital input path of analog pin DAC0_OUT and DAC1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12) | (1ul << 13));

    /* Single Mode test */
    /* Set the software trigger, enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);
    DAC_Open(DAC1, 0, DAC_SOFTWARE_TRIGGER);

    /* Enable DAC to work in group mode, once group mode enabled, DAC1 is configured by DAC0 registers */
    DAC_ENABLE_GROUP_MODE(DAC0);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, g_au16Sine[0]);
    DAC_WRITE_DATA(DAC1, 0, g_au16Sine[g_u32ArraySize / 2]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC0, 0);
    DAC_ENABLE_INT(DAC1, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC_START_CONV(DAC0);

    while(1);

}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
