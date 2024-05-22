/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code and execute in SRAM to program embedded Flash.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define TOTAL_VECTORS   (144)                               /* Total vector numbers */
__ALIGNED(256) uint32_t g_au32Vector[TOTAL_VECTORS] = {0};  /* Vector space in SRAM */

extern int32_t FlashAccess_OnSRAM(void);

volatile uint32_t g_u32Ticks = 0;
void SysTick_Handler()
{
    g_u32Ticks++;

    if((g_u32Ticks % 1000) == 0)
    {
        printf("Time elapse: %d\n", g_u32Ticks / 1000);
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}


int main(void)
{
    int32_t i;
    uint32_t *au32Vectors = (uint32_t *)0x0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Vector Table to SRAM */
    for(i = 0; i < TOTAL_VECTORS; i++)
    {
        g_au32Vector[i] = au32Vectors[i];
    }
    SCB->VTOR = (uint32_t)&g_au32Vector[0];

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /* SysTick used for test interrupts in SRAM */
    SysTick_Config(SystemCoreClock / 1000);

    /*
       This sample code demonstrates how to make a sub-routine code executed in SRAM.
    */

    printf("Will branch to address: 0x%x\n", (uint32_t)FlashAccess_OnSRAM);

    if(FlashAccess_OnSRAM())
    {
        printf("Flash access return error.\n");
    }
    else
    {
        printf("Flash access return ok.\n");
    }

    printf("\nFMC Sample Code Completed.\n");

    while(1);

}
