/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an I2S demo for recording data and demonstrate how I2S works with PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2S_TXData_DMA_CH 1
#define I2S_RX_DMA_CH 2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN 16
#define TX_BUFF_LEN 32

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

static DESC_TABLE_T s_asDescTable_DataTX[1], s_asDescTable_RX[2];

/* Global variable declaration */
static volatile uint8_t s_u8RxIdx = 0;
static uint32_t s_au32PcmRxDataBuff[2][BUFF_LEN] = {{0}};
static uint32_t s_au32PcmTxDataBuff[1][TX_BUFF_LEN] = {{0}};

/* Since ping-pong buffer would record infinitely, in this case we only want to make sure the first buffer of RX Buffer1 and Buffer2 are correct.
   Hence use s_au32PcmRxBuff and s_u8Count to check and record the first buffer of RX Buffer1 and Buffer2 */
static uint32_t s_au32PcmRxBuff[2][BUFF_LEN] = {{0}};
static volatile uint8_t s_u8Count = 0;

/* Function prototype declaration */
void PDMA_ResetRxSGTable(uint8_t id);
void SYS_Init(void);
void PDMA0_IRQHandler(void);

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    s_asDescTable_RX[id].CTL |= PDMA_OP_SCATTER;
    s_asDescTable_RX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32InitValue, u32DataCount, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+-----------------------------------------------+\n");
    printf("|       SPII2S + PDMA  Record Sample Code       |\n");
    printf("+-----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x50005000, 0x50015001, ... \n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PA3)\n      I2S_BCLK (PA2)\n");
    printf("      I2S_DI (PA1)\n      I2S_DO (PA0)\n\n");
    printf("      This sample code will transmit and receive data with PDMA transfer.\n");
    printf("      Connect I2S_DI and I2S_DO to check if the data which stored in two receive\n buffers are the same with the transmitted values.\n");
    printf("      After PDMA transfer is finished, the received values will be printed.\n\n");

    /* Enable I2S TX and RX functions */
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, 16000, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    /* Data initiation */
    u32InitValue = 0x50005000;
    for(u32DataCount = 0; u32DataCount < TX_BUFF_LEN; u32DataCount++)
    {
        s_au32PcmTxDataBuff[0][u32DataCount] = u32InitValue;
        u32InitValue += 0x00010001;
    }

    /* Enable PDMA channels */
    PDMA_Open(PDMA0, (1 << I2S_TXData_DMA_CH) | (1 << I2S_RX_DMA_CH));

    /* Tx description */
    s_asDescTable_DataTX[0].CTL = ((TX_BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_BASIC;
    s_asDescTable_DataTX[0].SA = (uint32_t)&s_au32PcmTxDataBuff[0];
    s_asDescTable_DataTX[0].DA = (uint32_t)&SPI0->TX;

    /* Rx(Record) description */
    s_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    s_asDescTable_RX[0].DA = (uint32_t)&s_au32PcmRxDataBuff[0];
    s_asDescTable_RX[0].FIRST = (uint32_t)&s_asDescTable_RX[1] - (PDMA0->SCATBA);

    s_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    s_asDescTable_RX[1].DA = (uint32_t)&s_au32PcmRxDataBuff[1];
    s_asDescTable_RX[1].FIRST = (uint32_t)&s_asDescTable_RX[0] - (PDMA0->SCATBA);   //link to first description

    PDMA_SetTransferMode(PDMA0, 1, PDMA_SPI0_TX, 1, (uint32_t)&s_asDescTable_DataTX[0]);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_SPI0_RX, 1, (uint32_t)&s_asDescTable_RX[0]);

    /* Enable PDMA channel 2 interrupt */
    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Clear RX FIFO */
    SPII2S_CLR_RX_FIFO(SPI0);

    /* Enable RX function and TX function */
    SPII2S_ENABLE_RX(SPI0);
    SPII2S_ENABLE_TX(SPI0);

    /* Enable RX PDMA and TX PDMA function */
    SPII2S_ENABLE_TXDMA(SPI0);
    SPII2S_ENABLE_RXDMA(SPI0);

    /* wait RX Buffer 1 and RX Buffer 2 get first buffer */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(s_u8Count != 2)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            goto lexit;
        }
    }

    /* Once I2S is enabled, CLK would be sent immediately, we still have chance to get zero data at beginning,
       in this case we only need to make sure the on-going data is correct */
    printf("RX Buffer 1\tRX Buffer 2\n");

    /* Print the received data */
    for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%X\t\t0x%X\n", s_au32PcmRxBuff[0][u32DataCount], s_au32PcmRxBuff[1][u32DataCount]);
    }

    printf("\n\nExit I2S sample code.\n");

lexit:

    while(1);

}

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

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

    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Configure SPI0 related multi-function pins. */
    /* GPA[3:0] : SPI0_SS (I2S0_LRCLK), SPI0_CLK (I2S0_BCLK), SPI0_MISO (I2S0_DI), SPI0_MOSI (I2S0_DO). */
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();
    SET_SPI0_CLK_PA2();
    SET_SPI0_SS_PA3();

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
}

void PDMA0_IRQHandler(void)
{
    uint32_t u32DataCount = 0;
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & 0x2)
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x4)             /* channel 2 done */
        {
            /* record the first buffer */
            if(s_u8Count == 0)
            {
                for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    s_au32PcmRxBuff[0][u32DataCount] = s_au32PcmRxDataBuff[0][u32DataCount];
            }
            else if(s_u8Count == 1)
            {
                for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    s_au32PcmRxBuff[1][u32DataCount] = s_au32PcmRxDataBuff[1][u32DataCount];
            }

            ++s_u8Count;
            /* Reset PDMA Scatter-Gather table */
            PDMA_ResetRxSGTable(s_u8RxIdx);
            s_u8RxIdx ^= 1;
        }
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}
