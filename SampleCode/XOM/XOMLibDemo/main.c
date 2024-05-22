
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demo how to use XOM library
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "xomlib.h"

#define PLL_CLOCK       64000000

void SYS_Init(void);

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

int main(void)
{
    extern void func(void);
    int32_t NumArray[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();


    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to call XOM library.

        The XOM library is built by XOMLib project.
        User need to add include path of xomlib.h and add object file xomlib.lib to using XOM library built by XOMLib project.

    Note:
        xomlib.lib only contain API function pointers.
        XOM code must be download to XOM region before call them by xomlib.lib

    */

    printf("\n\n");
    printf("+-------------------------------------------+\n");
    printf("|  Demo how to use XOM library Sample Code  |\n");
    printf("+-------------------------------------------+\n");

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", XOM_Add(100, 200));
    printf(" 500 - 100 = %d\n", XOM_Sub(500, 100));
    printf(" 200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("1000 / 250 = %d\n", XOM_Div(1000, 250));

    printf("\n");
    printf("1 + 2 +..+ 10 = %d\n", XOM_Sum(NumArray, 10));

    while(1);
}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
