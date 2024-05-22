/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to PDMA and trigger DAC by Timer.
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

void SYS_Init(void);


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

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable PDMA0 clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    //SET_UART0_RXD_PB12();//conflict with DAC0_OUT pin
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for DAC voltage output */
    SET_DAC0_OUT_PB12();

    /* Disable digital input path of analog pin DAC0_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12));

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
    printf("This sample code use PDMA and trigger DAC0 output sine wave by Timer 0.\n");

    /* Open Channel 0 */
    PDMA_Open(PDMA0, 0x1);

    /* Set transfer data width, and transfer count */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, g_u32ArraySize);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&g_au16Sine[0], PDMA_SAR_INC, (uint32_t)&DAC0->DAT, PDMA_DAR_FIX);

    /* Select channel 0 request source from DAC */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_DAC0_TX, FALSE, 0);

    /* Set transfer type and burst size */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_128);

    /* Set the timer 0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_TIMER0_TRIGGER);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the PDMA Mode */
    DAC_ENABLE_PDMA(DAC0);

    /* Enable Timer0 counting to start D/A conversion */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_DAC);
    TIMER_Start(TIMER0);

    while(1)
    {
        if(PDMA_GET_TD_STS(PDMA0) == 0x1)
        {
            /* Re-Set transfer count and basic operation mode */
            PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, g_u32ArraySize);
            PDMA_SetTransferMode(PDMA0, 0, PDMA_DAC0_TX, FALSE, 0);

            /* Clear PDMA channel 0 transfer done flag */
            PDMA_CLR_TD_FLAG(PDMA0, 0x1);
        }
    }
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
