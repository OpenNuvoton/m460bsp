/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate TIMER PWM accumulator interrupt to trigger PDMA transfer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32IsTestOver = 0;


/**
 * @brief       PDMA IRQ Handler
 * @param       None
 * @return      None
 * @details     The DMA default IRQ, declared in startup_m460.s.
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)        /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)   /* done */
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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
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
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    
    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    
    /* Set Timer0 PWM CH0(TM0) pin */
    SET_TM0_PB5();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32InitPeriod, u32UpdatedPeriod, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------------------------------+\n");
    printf("|    Timer PWM Accumulator Inerrupt Trigger PDMA Sample Code    |\n");
    printf("+---------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Timer0 PWM accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, Timer0 PWM period will be updated to (Initial Period x 2) by PDMA.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");
    printf("  Output frequency will be updated from 18kHz to 9kHz, and duty cycle from 50%% to 25%%.\n");

    printf("\n\nPress any key to start Timer0 PWM.\n\n");
    getchar();

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set Timer0 PWM mode as independent mode */
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);
    u32InitPeriod = TPWM_GET_PERIOD(TIMER0);

    /* Set Timer0 PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_DOWN_COUNT);

    /* Enable output of Timer0 PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable Timer0 PWM accumulator function, interrupt count 10, accumulator source select to zero point */
    TPWM_EnableAcc(TIMER0, 10, TPWM_IFA_ZERO_POINT);

    /* Enable Timer0 PWM accumulator interrupt trigger PDMA */
    TPWM_EnableAccPDMA(TIMER0);

    /* Enable Timer0 PWM interrupt */
    NVIC_EnableIRQ(TMR0_IRQn);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to TIMER PWM                              */
    /*--------------------------------------------------------------------------------------*/
    /* Open PDMA Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 1);

    /* Set updated period vaule */
    u32UpdatedPeriod = ((u32InitPeriod + 1) * 2) - 1;
    
    /* Set source address as u32UpdatedPeriod(no increment) and destination address as Timer0 PWM period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&u32UpdatedPeriod, PDMA_SAR_FIX, (uint32_t)&(TIMER0->PWMPERIOD), PDMA_DAR_FIX);

    /* Select PDMA request source as PDMA_TMR0(Timer0 PWM accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_TMR0, FALSE, 0);

    /* Set PDMA as single request type for Timer0 PWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_1);

    /* Enable PDMA interrupt */
    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Start Timer0 PWM counter */
    TPWM_START_COUNTER(TIMER0);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (g_u32IsTestOver != 1)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            return -1;
        }
    }

    printf("Timer0 PWM period register is updated from %d to %d\n\n", u32InitPeriod, TPWM_GET_PERIOD(TIMER0));

    printf("Press any key to stop Timer0 PWM.\n\n");
    getchar();

    /* Disable PDMA function */
    PDMA_Close(PDMA0);
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Stop Timer0 PWM */
    TPWM_STOP_COUNTER(TIMER0);

    /* Wait until Timer0 PWM Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((TIMER0->PWMCNT & TIMER_PWMCNT_CNT_Msk) != 0)
        if(--u32TimeOutCnt == 0) break;

    if(u32TimeOutCnt == 0)
        printf("Wait for Timer PWM stop time-out!\n");        
    else
        printf("Timer0 PWM has STOP.\n");

    while(1) {}
}
