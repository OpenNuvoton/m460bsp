/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate BMC data transfer with PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 22

/* 5b Table for PDMA transfer */
volatile uint32_t g_au32FiveBTable_GROUP0[TEST_COUNT] =
{
    0x00000000, 0x01010101, 0x02020202, 0x03030303, 0x04040404, 0x05050505, 0x06060606, 0x07070707, 0x08080808, 0x09090909, 0x0A0A0A0A,
    0x0B0B0B0B, 0x0C0C0C0C, 0x0D0D0D0D, 0x0E0E0E0E, 0x0F0F0F0F, 0x10101010, 0x1F1F1F1F, 0x1A1A1A1A, 0x15151515, 0x19191919, 0x14141414
};

/* Function prototype declaration */
void SYS_Init(void);
void BMC_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable BMC module clock */
    CLK_EnableModuleClock(BMC_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Configure BMC related multi-function pins for group 0. */
    SET_BMC0_PB5();
    SET_BMC1_PB4();
    SET_BMC2_PB3();
    SET_BMC3_PB2();

    /* Enable BMC I/O schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk | GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}

void BMC_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init BMC                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure BMC */
    /* Configure BMC bit clock rate 2MHz, set 64 bits preamble, set the bit time period of logic 0 is same as logic 1. */
    BMC_SetBitClock(2000000);
    BMC_PREAMBLE_BIT(BMC_PREAMBLE_64);
    BMC_BITWIDTH_ADJUST(BMC_BITWIDTH_1);

    /* Set the dummy level to high and the dummy delay time of group 0 to 1000us. Enable BMC group 0 channels. */
    BMC_DUMMY_LEVEL(BMC_DUM_LVL_HIGH);
    BMC_SetDummyDelayPeriod(BMC_GROUP_0, 1000);
    BMC_ENABLE_GROUP0();
}

int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init BMC */
    BMC_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------------+\n");
    printf("|              BMC + PDMA Sample Code              |\n");
    printf("+--------------------------------------------------+\n");
    printf("BMC output pins:\n");
    printf("    CH0(PB5), CH1(PB4), CH2(PB3), CH3(PB2)\n");
    printf("Press any key to start transmission ...");
    getchar();
    printf("\n");

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Open Channel 0 */
    PDMA_Open(PDMA0, 1 << 0);
    /* Transfer count is TEST_COUNT, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32FiveBTable_GROUP0, PDMA_SAR_INC, (uint32_t)&BMC->TXDATG0, PDMA_DAR_FIX);
    /* Request source is BMC_G0_TX */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_BMC_G0_TX, FALSE, 0);
    /* Transfer type is single transfer and burst size is 4 (No effect in single request type) */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_4);

    /* Enable PDMA function */
    BMC_ENABLE_DMA();

    /* Enable BMC controller */
    BMC_ENABLE();

    /* Wait until one frame transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!BMC_GetIntFlag(BMC_FTXD_INT_MASK))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BMC interrupt flag time-out!\n");
            break;
        }
    }

    /* Disable PDMA function */
    BMC_DISABLE_DMA();

    printf("\n\nExit BMC driver sample code.\n");

    while(1);
}
