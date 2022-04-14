/*************************************************************************//**
 * @file     isr.c
 * @version  V3.00
 * @brief    ISR source file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "l3.h"

#include "config.h"

extern volatile uint8_t aPCMBuffer_Full[2];
volatile uint8_t u8PCMBuffer_Playing = 0;
uint32_t volatile g_u32BuffPos = 0;
extern volatile uint32_t u32RecordDone;

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x2)    /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x4)              /* channel 2 done */
        {
            if(aPCMBuffer_Full[u8PCMBuffer_Playing ^ 1] != 1)
                printf("underflow!!\n");
            aPCMBuffer_Full[u8PCMBuffer_Playing] = 0;       /* Set empty flag */
            u8PCMBuffer_Playing ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }
    }
    else if(u32Status & 0x400)     /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        printf("PDMA Timeout!!\n");
    }
    else
    {
        printf("0x%x\n", u32Status);
        while(1);
    }
}

void I2S0_IRQHandler(void)
{
    uint32_t u32Reg;
    uint32_t u32Len, i;

    u32Reg = I2S_GET_INT_FLAG(I2S0, I2S_STATUS0_RXTHIF_Msk);

    if(u32Reg & I2S_STATUS0_RXTHIF_Msk)
    {
        /* Read Rx FIFO Level */
        u32Len = I2S_GET_RX_FIFO_LEVEL(I2S0);

        for(i = 0; i < u32Len; i++)
        {
            outp32(HYPER_RAM_MEM_MAP + g_u32BuffPos + 4 * i, I2S_READ_RX_FIFO(I2S0));
        }

        g_u32BuffPos += 4 * u32Len;

        if(g_u32BuffPos >= 0x700000)
        {
            u32RecordDone = 1;
            I2S_DISABLE_RX(I2S0);
            printf("The record data size reaches the allocated HyperRAM size of 7MBytes.\nStop recording ...\n");
        }
    }
}
