/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate EPWM accumulator interrupt trigger PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint16_t g_au16Period[2] = {31999, 15999};
static volatile uint32_t g_u32IsTestOver = 0;

void PDMA0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       PDMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PDMA interrupt event
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
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

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for EPWM */
    SET_EPWM1_CH0_PC5();

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
    uint32_t u32NewCNR = 0;
    uint32_t u32TimeOutCnt = 0;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                         EPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrate EPWM1 channel 0 accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, EPWM1 channel 0 period register will be updated \n");
    printf("  to g_u32Count array content, 31999(0x7CFF), by PDMA.\n");

    printf("\n\nPress any key to start EPWM1 channel 0.\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the EPWM1 Channel 0 as EPWM output function.                                     */
    /*--------------------------------------------------------------------------------------*/

    /* Set EPWM1 channel 0 output configuration */
    EPWM_ConfigOutputChannel(EPWM1, 0, 300, 30);

    /* Enable EPWM Output path for EPWM1 channel 0 */
    EPWM_EnableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Enable EPWM1 channel 0 accumulator, interrupt count 50, accumulator source select to zero point */
    EPWM_EnableAcc(EPWM1, 0, 50, EPWM_IFA_ZERO_POINT);

    /* Enable EPWM1 channel 0 accumulator interrupt */
    EPWM_EnableAccInt(EPWM1, 0);

    /* Enable EPWM1 channel 0 accumulator interrupt trigger PDMA */
    EPWM_EnableAccPDMA(EPWM1, 0);

    /* Enable Timer for EPWM1 channel 0 */
    EPWM_Start(EPWM1, EPWM_CH_0_MASK);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to EPWM                                   */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 1);

    /* Set source address as g_u32Count array(increment) and destination address as EPWM1 channel 0 period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&g_au16Period[0], PDMA_SAR_INC, (uint32_t) & (EPWM1->PERIOD[0]), PDMA_DAR_FIX);

    /* Select PDMA request source as EPWM1_CH0_TX(EPWM1 channel 0 accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_EPWM1_CH0_TX, FALSE, 0);

    /* Set PDMA as single request type for EPWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_1);

    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver != 1)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            goto lexit;
        }
    }

    u32NewCNR = EPWM_GET_CNR(EPWM1, 0);
    printf("\n\nEPWM1 channel0 period register is updated to %d(0x%x)\n", u32NewCNR, u32NewCNR);
    printf("Press any key to stop EPWM1 channel 0.\n");
    getchar();

    /* Set EPWM1 channel 0 loaded value as 0 */
    EPWM_Stop(EPWM1, EPWM_CH_0_MASK);

    /* Wait until EPWM1 channel 0 Timer Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((EPWM1->CNT[0] & EPWM_CNT0_CNT_Msk) != 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EPWM stop time-out!\n");
            break;
        }
    }

lexit:

    /* Disable Timer for EPWM1 channel 0 */
    EPWM_ForceStop(EPWM1, EPWM_CH_0_MASK);

    /* Disable EPWM Output path for EPWM1 channel 0 */
    EPWM_DisableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Disable PDMA NVIC */
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Close PDMA */
    PDMA_Close(PDMA0);

    while(1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
