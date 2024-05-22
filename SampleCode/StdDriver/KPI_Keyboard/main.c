/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set scan key board by KPI.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//*** <<< Use Configuration Wizard in Context Menu >>> ***
//<c0> Use Internal Pull-up
//#define USE_INTERNAL_PULLUP
//</c>
//*** <<< end of configuration section >>>    ***




void SYS_Init(void);
void UART0_Init(void);

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
    CLK_EnableModuleClock(KPI_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(KPI_MODULE, CLK_CLKSEL3_KPISEL_HIRC, CLK_CLKDIV2_KPI(0));


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
    KPI_KEY_T queue[32] = {0};
    KPI_KEY_T key;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------------------+\n");
    printf("|          Keypad Interface Sample Code           |\n");
    printf("+-------------------------------------------------+\n\n");
    printf("  Key array board is necessary for this sample code\n");

    SET_KPI_ROW0_PC5();
    SET_KPI_ROW1_PC4();
    SET_KPI_COL0_PB15();


    /* Init KPI */
    KPI_Open(2, 1, queue, 32);
    /* Key sampling time */
    KPI_SetSampleTime(10);
#ifdef USE_INTERNAL_PULLUP
    CLK_EnableModuleClock(GPB_MODULE);
    GPIO_SetPullCtl(PB, BIT15, GPIO_PUSEL_PULL_UP);
    /* Interal pull-up is weak and needs to slowdown the timing */
    KPI_EnableSlowScan();
#endif

    /* Clear status */
    KPI->STATUS = KPI->STATUS;
    while(1)
    {
        /* Check if any key pressed */
        if(KPI_kbhit())
        {
            key = KPI_GetKey();
            printf("%d, %d, %s\n", key.x, key.y, (key.st == KPI_PRESS) ? "PRESS" : "RELEASE");
        }
    }

}
