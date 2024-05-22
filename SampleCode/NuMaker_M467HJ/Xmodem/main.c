/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to transfer data through UART by Xmodem.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "xmodem.h"

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

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

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

int main(void)
{
    int32_t i32Err;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|         Xmodem Sample Code             |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    printf("Waiting for Xmodem data transfer ...\n");

    /* Waiting for debug message print out */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) == 0)
        if(--u32TimeOutCnt == 0) break;

    i32Err = Xmodem(0x80000);

    if(i32Err < 0)
    {
        printf("Xmodem transfer fail![0x%X]\n", i32Err);
    }
    else
    {
        printf("Xomdem transfer done!\n");
        printf("Total trnasfer size is %d\n", i32Err);

        printf("press any key to read back the transfer file.\n");
        getchar();
        printf("Waiting for receiver ...\n");


        i32Err = XmodemSend((uint8_t *)0x80000, i32Err);

        if(i32Err < 0)
        {
            printf("Xmodem transfer fail!\n");
        }
        else
        {
            printf("Transfer done.\nTotal trasnfer size is %d\n", i32Err);
        }
    }

    while(1);

}
