/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set different core voltage.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



static volatile uint8_t s_u8IsINTEvent;

void WDT_IRQHandler(void);
void PowerDownFunction(void);
int32_t pi(void);
void CheckSystemWork(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{

    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    s_u8IsINTEvent = 1;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t s_ai32f[PI_NUM + 1];
static uint32_t s_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t s_ai32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for(; b - c;)
        s_ai32f[b++] = a / 5;

    i = 0;
    for(; (void)(d = 0), g = c * 2; c -= 14, s_ai32piResult[i++] = e + d / a, e = d % a)
    {
        if(i == 19)
            break;

        for(b = c; (void)(d += s_ai32f[b] * a), (void)(s_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }
    i32Err = 0;
    for(i = 0; i < 19; i++)
    {
        if(s_au32piTbl[i] != (uint32_t)s_ai32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void CheckSystemWork(void)
{
    if(pi())
    {
        printf("[FAIL]\n");
    }
    else
    {
        printf("[OK]\n");
    }
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

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);

    /* Select WDT module clock source as LIRC */
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|         Power Mode Sample Code        |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers before setting power level and entering Idle mode */
    SYS_UnlockReg();

    /* Set core clock as 180MHz from PLL */
    CLK_SetCoreClock(180000000);

    /* Set power level to 1.2V */
    printf("Set power level to 1.2V ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL1);

    /* Check system work */
    CheckSystemWork();

    /* Set power level to 1.26V */
    printf("Set power level to 1.26V ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL0);

    /* Set core clock as 200MHz from PLL */
    CLK_SetCoreClock(200000000);

    /* Check system work */
    CheckSystemWork();

    /* Enter to Idle mode and wake-up by WDT */
    printf("Enter to Idle mode and wake-up ");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enter to Idle mode */
    CLK_Idle();

    /* Check if WDT time-out interrupt occurred or not */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(s_u8IsINTEvent == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for WDT interrupt time-out!\n");
            break;
        }
    }

    /* Check system work */
    CheckSystemWork();

    printf("Sample code end.\n");

    while(1);

}
