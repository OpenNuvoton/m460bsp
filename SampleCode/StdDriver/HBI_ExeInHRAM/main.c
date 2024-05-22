/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code to execute in HyperRAM.
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

extern int32_t ExeInHRAM(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(180000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update system core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();



}

extern uint32_t Load$$HRAM$$RO$$Base;
extern uint32_t Load$$HRAM$$RO$$Limit;
extern uint32_t Load$$HRAM$$RO$$Length;
extern uint32_t Image$$HRAM$$RO$$Base;


//volatile uint32_t g_u32Ticks = 0;


int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+------------------------------------------------+\n");
    printf("|      Code execute in HyperRAM Sample Code      |\n");
    printf("+------------------------------------------------+\n");

    /*
       This sample code demonstrates how to make a sub-routine code executed in SRAM.

       Build option "HBI_ENABLE = 2" is defined to init HBI in SystemInit()
    */

    printf("Will branch to address: 0x%08x\n", (uint32_t)ExeInHRAM);

    ExeInHRAM();

    printf("\nReturn to execute in APROM \n");
    printf("\nHyperRAM execute code test done \n");

    while(1);

}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
