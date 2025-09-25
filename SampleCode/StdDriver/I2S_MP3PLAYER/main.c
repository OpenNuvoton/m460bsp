/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to plays MP3 bitstream.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"

#ifdef __ICCARM__
#pragma data_alignment=32
DMA_DESC_T DMA_DESC[2];
#else
DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
#endif

extern volatile uint32_t aPCMBuffer[2][PCM_BUFFER_SIZE];
extern uint32_t volatile sd_init_ok;

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
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
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
}

void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 100k */
    I2C_Open(I2C2, 100000);
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&aPCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA0->SCATBA);

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&aPCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA0->SCATBA);

    PDMA_Open(PDMA0, 1 << 2);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_I2S0_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Init GPIO to monitor the buffer usage */
    PH4 = 1; // LED_R
    PH5 = 0; // LED_Y
    PH6 = 0; // LED_G
    GPIO_SetMode(PH, BIT4 | BIT5 | BIT6, GPIO_MODE_OUTPUT);

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                   MP3 Player Sample with audio codec                  |\n");
    printf("+-----------------------------------------------------------------------+\n");

    /* Init I2C2 to access codec */
    I2C2_Init();

    /* Select source from HXT(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);

    while(1)
    {
        /* Play mp3 */
        MP3Player();
    }
}
