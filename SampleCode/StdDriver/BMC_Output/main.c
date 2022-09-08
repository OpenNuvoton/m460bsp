/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate BMC data transfer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 22

volatile uint32_t g_au32FrameTransmitDoneIntFlag = 0;
volatile uint32_t g_u32CountCH0 = 0, g_u32CountCH1 = 0, g_u32CountCH2 = 0, g_u32CountCH3 = 0;

/* 5b Table for FIFO transfer */
volatile uint8_t g_au8FiveBTable_CH0[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};
volatile uint8_t g_au8FiveBTable_CH1[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};
volatile uint8_t g_au8FiveBTable_CH2[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};
volatile uint8_t g_au8FiveBTable_CH3[TEST_COUNT] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x1F, 0x1A, 0x15, 0x19, 0x14
};

void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        if(BMC_GET_CH_EMPTY_FLAG() & BIT0)
        {
            outp8((uint32_t)&(BMC->TXDATG0) + 0x0, g_au8FiveBTable_CH0[g_u32CountCH0++]);
        }
        if(BMC_GET_CH_EMPTY_FLAG() & BIT1)
        {
            outp8((uint32_t)&(BMC->TXDATG0) + 0x1, g_au8FiveBTable_CH1[g_u32CountCH1++]);
        }
        if(BMC_GET_CH_EMPTY_FLAG() & BIT2)
        {
            outp8((uint32_t)&(BMC->TXDATG0) + 0x2, g_au8FiveBTable_CH2[g_u32CountCH2++]);
        }
        if(BMC_GET_CH_EMPTY_FLAG() & BIT3)
        {
            outp8((uint32_t)&(BMC->TXDATG0) + 0x3, g_au8FiveBTable_CH3[g_u32CountCH3++]);
        }
    }
}

void BMC_IRQHandler(void)
{
    if((BMC->INTSTS & BMC_INTSTS_FTXDIF_Msk) && (BMC->INTEN & BMC_INTEN_FTXDIEN_Msk))
    {
        /* Clear frame transmit done interrupt flag */
        BMC_ClearIntFlag(BMC_FTXD_INT_MASK);

        /* Disable Timer0 NVIC */
        NVIC_DisableIRQ(TMR0_IRQn);
        /* Stop Timer0 counting */
        TIMER_Stop(TIMER0);

        g_au32FrameTransmitDoneIntFlag += 1;
    }
}

/* Function prototype declaration */
void TMR0_IRQHandler(void);
void BMC_IRQHandler(void);
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

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

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
    /* Configure BMC bit clock rate 2MHz, set 64 bits preamble, set the bit time period of logic 0 is 1.5 times logic 1. */
    BMC_SetBitClock(2000000);
    BMC_PREAMBLE_BIT(BMC_PREAMBLE_64);
    BMC_BITWIDTH_ADJUST(BMC_BITWIDTH_15);

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
    printf("|                  BMC Sample Code                 |\n");
    printf("+--------------------------------------------------+\n");
    printf("BMC output pins:\n");
    printf("    CH0(PB5), CH1(PB4), CH2(PB3), CH3(PB2)\n");
    printf("Press any key to start transmission ...");
    getchar();
    printf("\n");

    /* Open Timer0 in periodic mode, enable interrupt */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 2000000);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Enable BMC transmit done interrupt */
    BMC_EnableInt(BMC_FTXD_INT_MASK);

    /* Enable BMC NVIC */
    NVIC_EnableIRQ(BMC_IRQn);

    /* BMC interrupt has higher frequency then TMR0 interrupt. */
    NVIC_SetPriority(TMR0_IRQn, 3);
    NVIC_SetPriority(BMC_IRQn, 2);

    BMC->TXDATG0 = (g_au8FiveBTable_CH0[g_u32CountCH0++] << BMC_TXDATG0_CH0_TXDAT_Pos) | (g_au8FiveBTable_CH1[g_u32CountCH1++] << BMC_TXDATG0_CH1_TXDAT_Pos) |
                   (g_au8FiveBTable_CH2[g_u32CountCH2++] << BMC_TXDATG0_CH2_TXDAT_Pos) | (g_au8FiveBTable_CH3[g_u32CountCH3++] << BMC_TXDATG0_CH3_TXDAT_Pos);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Enable BMC controller */
    BMC_ENABLE();

    /* Wait until one frame transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_au32FrameTransmitDoneIntFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BMC interrupt time-out!\n");
            break;
        }
    }

    printf("\n\nExit BMC driver sample code.\n");

    while(1);
}
