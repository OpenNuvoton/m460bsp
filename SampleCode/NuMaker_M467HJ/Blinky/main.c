/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple demo for NuMaker-M467HJ board to show message from UART0 to ICE VCOM and show LED
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "clk.h"
#include "uart.h"
#include "gpio.h"
#include "sys.h"


#define LED_INIT()  (PH->MODE = ((PH->MODE &(~(0x3ful << 4*2))) | (0x15ul << 4 *2)))
#define LED_RED     PH4
#define LED_YELLOW  PH5
#define LED_GREEN   PH6

void SYS_Init(void);
void UART0_Init(void);

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 192MHz */
    CLK_SetCoreClock(192000000);

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
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}



int main()
{
    int32_t i = 0;

    SYS_UnlockReg();

    SYS_Init();

    UART0_Init();

    /* Enable GPIO PH to control LED */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPHCKEN_Msk;

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|              Simple Blinky Demo                                  |\n");
    printf("+------------------------------------------------------------------+\n");

    /* Init GPIO for LED toggle */
    LED_INIT();
    LED_YELLOW = 1;
    LED_RED = 0;
    LED_GREEN = 0;
    while(1)
    {
        LED_YELLOW = 0;
        LED_RED = 1;
        LED_GREEN = 1;
        CLK_SysTickLongDelay(200000);
        LED_YELLOW = 1;
        LED_RED = 0;
        LED_GREEN = 1;
        CLK_SysTickLongDelay(200000);
        LED_YELLOW = 1;
        LED_RED = 1;
        LED_GREEN = 0;
        CLK_SysTickLongDelay(200000);
        printf("Iter %d\n", i++);
    }


}
