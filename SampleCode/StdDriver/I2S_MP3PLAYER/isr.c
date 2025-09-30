/*************************************************************************//**
 * @file     isr.c
 * @version  V3.00
 * @brief    ISR source file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"
#include "pdma.h"
#include "pdma_reg.h"

extern uint32_t aPCMBuffer[2][PCM_BUFFER_SIZE];

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);
    if(u32Status & 0x2)    /* done */
    {
        /* Monitor SA register to check PDMA is working on which buffer */
        if(PDMA0->DSCT[2].SA == (uint32_t)&aPCMBuffer[1][0])
        {
            /* Buffer 0 is empty */
            PH4 = 1;
        }
        else
        {
            /* Buffer 1 is empty */
            PH5 = 1;
        }

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else if(u32Status & 0x400)     /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        printf("PDMA Timeout!!\n");
    }
    else if(u32Status & PDMA_INTSTS_ABTIF_Msk)
    {
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else
    {

        //printf("0x%x\n", u32Status);
        //while(1);
    }
}
