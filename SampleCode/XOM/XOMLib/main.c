/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to config/erase XOM region.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "xomapi.h"



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
    char ch;
    int32_t i, r;


    int32_t NumArray[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();


    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to build an XOM libary.

        The location of XOM region is defined by scatter file: xom_scatter.scf
        The API header file is xomapi.h
        The XOM functions are implemented in xom.c

        This project is only used to build code for XOM region and test its functions.
        To enable XOM region, please use "NuMicro ICP Programming Tool".



        Example flow:
        1. Build XOMCode and test XOM functions
        2. Open "NuMicro ICP Programming Tool" to enable XOM region and
           according to xom_scatter.scf settings.
        3. Test XOM function with XOM enabled again.
        4. Review xomlib.c(Keil), xomlibIAR.c(IAR) and .\lib\xomlib.h
           to make sure all XOM function pointers are included.
        5. (Keil) Build final XOMCode. XOMAddr.exe will be executed to update
                  function pointer addresses after built.
           (IAR) Update xomlibIAR.c function pointers manually.
        6. Build XOMLib project to generate xomlib.lib or xomlib.a.
           It includes function pointers for XOM.
           The library (xomlib.lib or xomlib.a) and header (xomlib.h) is located at lib directory.
        7. Pass xomlib library & xomlib.h to the people who will call the functions in XOM.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      FMC XOM Library Build Example     |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active*/
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    printf("[0] Set XOM\n");
    printf("[1] Test XOM\n");

    ch = (char)getchar();

    if(ch == '0')
    {
        FMC_ConfigXOM(0, 0x10000, 1);
    }




    /* Read User Configuration */
    printf("\n");
    printf("XOM Status = 0x%X\n", FMC->XOMSTS);

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", XOM_Add(100, 200));
    printf(" 500 - 100 = %d\n", XOM_Sub(500, 100));
    printf(" 200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("1000 / 250 = %d\n", XOM_Div(1000, 250));

    printf("\n");
    printf("1 + 2 +..+ 10 = %d\n", XOM_Sum(NumArray, 10));


    for(i = 0; i < 1000; i++)
    {
        r = XOM_Add(500, 700);
        if(r != 1200)
        {
            printf("XOM ADD fail. It should be 1200 but %d\n", r);
            goto lexit;
        }
    }

    for(i = 0; i < 16; i++)
    {
        printf("[%04x] = 0x%08x\n", 0x10000 + i * 4, M32(0x10000 + i * 4));
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
